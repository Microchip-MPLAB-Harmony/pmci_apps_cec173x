/*****************************************************************************
* Copyright (c) 2022 Microchip Technology Inc. and its subsidiaries.
* You may use this software and any derivatives exclusively with
* Microchip products.
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".
* NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
* INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
* AND FITNESS FOR A PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP
* PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.
* TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL
* CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF
* FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
* MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE
* OF THESE TERMS.
*****************************************************************************/

#include "definitions.h"

#include "mctp.h"
#include "mctp_common.h"
#include "mctp_base.h"
#include "mctp_smbus.h"
#include "mctp_control.h"
#include "mctp_config.h"

/* function declarations */
void mctp_init_buffers(void);
uint8_t mctp_tx_timeout(MCTP_PKT_BUF *tx_buf);
void mctp_event_tx_handler(void);
void mctp_event_rx_handler(void);
void mctp_handle_ec_rx_request_pkt(void);

MCTP_BSS_ATTR MCTP_PKT_BUF mctp_pktbuf[MCTP_PKT_BUF_NUM]__attribute__ ((aligned(8)));

/* mctp tx state machine variables */
MCTP_BSS_ATTR uint8_t mctp_tx_state;
MCTP_BSS_ATTR uint8_t mctp_txbuf_index;
MCTP_BSS_ATTR uint8_t mctp_wait_smbus_callback;
MCTP_BSS_ATTR uint8_t active_pkt_msg_type_rx; // pldm or spdm or mctp
MCTP_BSS_ATTR uint8_t msg_type_tx; // pldm or spdm or mctp - when transmitting multiple/single pkt through smbus
MCTP_BSS_ATTR static uint8_t out_of_seq_detected;

/* extern variables */
MCTP_BSS_ATTR struct MCTP_CFG_PARA mctp_cfg;
MCTP_BSS_ATTR struct MCTP_IDENTITY mctp_rx[MCTP_MSG_CONTEXT]; //currently supporting SPDM and PLDM application
MCTP_BSS_ATTR struct MCTP_TX_CXT mctp_tx[MCTP_MSG_CONTEXT];
MCTP_BSS_ATTR static uint8_t cmd_field;

MCTP_BSS_ATTR static uint8_t transmit_buf[sizeof(MCTP_BUFDATA)]__attribute__ ((aligned(8)));

MCTP_BSS_ATTR static uint32_t start_time;


/******************************************************************************/
/** UPDATES packetizing variable to check if input data spans more than one mctp packet
* @param NULL
* @return true/false
*******************************************************************************/
bool mctp_base_packetizing_val_get(uint8_t msg_type)
{
    uint8_t i =0;
    for (i =0; i < 2; i++)
    {
        if (mctp_rx[i].message_type == msg_type)
        {
            return mctp_rx[i].packetizing;
        }
    }
    return 0;
}

/******************************************************************************/
/** UPDATES packetizing variable to check if input data spans more than one mctp packet
* @param NULL
* @return true/false
*******************************************************************************/
void mctp_base_packetizing_val_set(uint8_t msg_type, bool value)
{
    uint8_t i =0;
    for (i =0; i < 2; i++)
    {
        if (mctp_rx[i].message_type == msg_type)
        {
            mctp_rx[i].packetizing = value;
        }
    }
    return;
}
/******************************************************************************/
/** UPDATES CURRENT_EID FIELD OF RESPECTIVE ENDPOINT OF MCTP BRIDGE ROUTING TABLE
* @param i Endpoint entry id
* @return void
*******************************************************************************/
void mctp_rtupdate_current_eid(uint8_t i)
{
    if(mctp_rt.epA.ep[i].field.default_eid != 0U) /* means static eid config */
    {
        mctp_rt.epA.ep[i].field.current_eid = mctp_rt.epA.ep[i].field.default_eid;
    }

} /* End mctp_rtupdate_current_eid */

/******************************************************************************/
/** UPDATES EID_TYPE FIELD OF RESPECTIVE ENDPOINT OF MCTP BRIDGE ROUTING TABLE
* @param i Endpoint entry id
* @return void
*******************************************************************************/
void mctp_rtupdate_eid_type(uint8_t i)
{
    if(mctp_rt.epA.ep[i].field.default_eid == 0U) /* means dynamic eid config */
    {
        mctp_rt.epA.ep[i].field.eid_type = (uint8_t)MCTP_DYNAMIC_EID;
    }
    else if(mctp_rt.epA.ep[i].field.default_eid != 0U) /* means static eid config */
    {
        if(mctp_rt.epA.ep[i].field.current_eid == mctp_rt.epA.ep[i].field.default_eid)
        {
            mctp_rt.epA.ep[i].field.eid_type = (uint8_t)MCTP_STATIC_DEQC;
        }
        else if(mctp_rt.epA.ep[i].field.current_eid != mctp_rt.epA.ep[i].field.default_eid)
        {
            mctp_rt.epA.ep[i].field.eid_type = (uint8_t)MCTP_STATIC_DNEQC;
        }
        else
        {
            /* Invalid eid */;
        }
    }
    else
    {
        /* Invalid config */;
    }

} /* End mctp_rtupdate_eid_type */

/******************************************************************************/
/** UPDATES EID_STATE FIELD OF RESPECTIVE ENDPOINT OF MCTP BRIDGE ROUTING TABLE
* @param i Endpoint entry id
* @return void
*******************************************************************************/
void mctp_rtupdate_eid_state(uint8_t i)
{
    if(mctp_rt.epA.ep[i].field.eid_type != (uint8_t)MCTP_DYNAMIC_EID) /* means static */
    {
        mctp_rt.epA.ep[i].field.eid_state = (uint8_t)MCTP_STATIC_ASSIGNED;
    }
    else if(mctp_rt.epA.ep[i].field.eid_type == (uint8_t)MCTP_DYNAMIC_EID) /* means dynamic */
    {
        if(mctp_rt.epA.ep[i].field.current_eid == 0x00U)
        {
            mctp_rt.epA.ep[i].field.eid_state = (uint8_t)MCTP_DYNAMIC_NOTASSIGNED;
        }
        else
        {
            mctp_rt.epA.ep[i].field.eid_state = (uint8_t)MCTP_DYNAMIC_ASSIGNED;
        }
    }
    else
    {
        /* Invalid eid */;
    }

} /* End mctp_rtupdate_eid_state */

/******************************************************************************/
/** Initializes the mctp module.
* Mainly does initialization specific to mctp buffers, buffer parameters,
* variables and mctp-smbus/emi interface.
* @param void
* @return void
*******************************************************************************/
void mctp_init_task(void)
{
    /* initialize mctp buffers */
    mctp_init_buffers();
    mctp_clean_up_buffer_states();

    /* initialization specific to smbus */
    //For data isolation - Register slave application function with smbus driver not required
    //smb_status = mctp_smbus_init();

    /* initialize mctp specific variables */
    mctp_tx_state = (uint8_t)MCTP_TX_IDLE;
    mctp_txbuf_index = MCTP_BUF1; // Used for TX
    mctp_wait_smbus_callback = 0U;
    mctp_cfg.smbus_fairness = 1U; /// Fairness enable
    mctp_cfg.smbus_speed = (uint8_t)MCTP_I2C_CLK_FREQ; //default set to 400 Khz
    mctp_cfg.mctp_discovery = (uint8_t)false;
} /* End mctp_init_task(void) */

/******************************************************************************/
/** Store I2C Physical layer parameters into MCTP context structure
* @param void
* @return void
*******************************************************************************/
void mctp_update_i2c_params(MCTP_CONTEXT* ret_mctp_ctxt)
{
    uint8_t i;
    uint8_t mctp_eid;
    if(NULL == ret_mctp_ctxt)
    {
        return;
    }
    mctp_cfg.smbus_speed = ret_mctp_ctxt->i2c_bus_freq;

    /* update mctp bridging routing table endpoint entries */
    for(i = 0U; i < (uint8_t)MCTP_ENDPOINTS_MAX; i++)
    {
        mctp_rtupdate_current_eid(i);
        mctp_rtupdate_eid_type(i);
        mctp_rtupdate_eid_state(i);
    }
    mctp_eid = ret_mctp_ctxt->eid;

    mctp_rt.epA.ep[MCTP_RT_EC_INDEX].field.default_eid = mctp_eid;
    mctp_rt.epA.ep[MCTP_RT_EC_INDEX].field.current_eid = mctp_eid;
    mctp_rtupdate_eid_type((uint8_t)MCTP_RT_EC_INDEX);
    mctp_rtupdate_eid_state((uint8_t)MCTP_RT_EC_INDEX);
}


/******************************************************************************/
/** Initializes mctp buffers.
* @param void
* @return void
*******************************************************************************/
void mctp_init_buffers(void)
{
    uint8_t i = 0;
    uint8_t j = 0;

    for(i = 0; i < MCTP_PKT_BUF_NUM; i++)
    {
        for(j = 0; j < MCTP_PKT_BUF_DATALEN; j++)
        {
            mctp_pktbuf[i].pkt.data[j] = 0;
        }

        mctp_pktbuf[i].buf_full = (uint8_t)MCTP_EMPTY;
        mctp_pktbuf[i].smbus_nack_retry_count = 0;
        mctp_pktbuf[i].smbus_acquire_retry_count = 0;
        mctp_pktbuf[i].smbus_lab_retry_count = 0;
        mctp_pktbuf[i].request_tx_retry_count = 0;
        mctp_pktbuf[i].request_per_tx_timeout_count = 0;
        mctp_pktbuf[i].rx_timestamp = 0;
    }

} /* End mctp_init_buffers() */

/******************************************************************************/
/** This is called whenever kernel schedules mctp event task.
* Mctp module calls SET_MCTP_EVENT_TASK(mctp) for scheduling mctp event task.
* This event task is called whenever packet is received over smbus, or
* packet is to be transmitted over smbus.
* @param void
* @return void
*******************************************************************************/
void mctp_event_task(void)
{
    // Handling packet transmission over smbus
    mctp_event_tx_handler();

    // Handling packet received over smbus
    mctp_event_rx_handler();

} /* End mctp_event_task() */

/******************************************************************************/
/** mctp_event_tx_handler is called from mctp_event_task for handling
* packet transmission over smbus and to handle mctp tx state machine.
* @param void
* @return void
*******************************************************************************/
void mctp_event_tx_handler(void)
{
    uint8_t index = 0x00;
    uint32_t interval = 0;
    uint32_t hdr_struct_size = sizeof(MCTP_HEADER);

    MCTP_CONTEXT *mctp_context = NULL;
    mctp_context = mctp_ctxt_get();
    if(NULL == mctp_context)
    {
        return;
    }

    MCTP_PKT_BUF *tx_buf = NULL;
    uint8_t acquire_status = 0x00;
    MCTP_TX_CXT *mctp_tx_ctxt = NULL;
    
    switch(mctp_tx_state)
    {
    case (uint8_t)MCTP_TX_IDLE:

        /* this will be only executed if there is mctp event set by rx;
         * nothing to be done here */

        break;

    case (uint8_t)MCTP_TX_NEXT:

        for(index = 0U; index < 5U; index++)
        {
            /* check if that tx buffer is valid */
            if((uint8_t)MCTP_TX_PENDING == mctp_pktbuf[mctp_txbuf_index].buf_full)
            {
                /* if TX buffer packet is response packet. As per the new spec,
                * check timing only for response packet from EC or MC*/
                if(mctp_txbuf_index == MCTP_BUF1 || mctp_txbuf_index == MCTP_BUF2 || mctp_txbuf_index == MCTP_BUF5)
                {
                    /* get current TX buffer pointer */
                    tx_buf = (MCTP_PKT_BUF *)&mctp_pktbuf[mctp_txbuf_index];
                    (void)memset(&transmit_buf[0], 0, (uint32_t)sizeof(MCTP_BUFDATA));

                    if ((bool)tx_buf->pkt.field.hdr.som == true)
                    {
                        mctp_tx_ctxt = mctp_msg_tx_ctxt_create(tx_buf->pkt.field.hdr.msg_type, tx_buf->pkt.field.hdr.src_eid,
                                            tx_buf->pkt.field.hdr.dst_eid, tx_buf->pkt.field.hdr.msg_tag);
                        
                        if (mctp_tx_ctxt != NULL) {
                        msg_type_tx = mctp_tx_ctxt->message_type;
                        }
                    }
                    if ((bool)tx_buf->pkt.field.hdr.som == false)
                    {
                        mctp_tx_ctxt = mctp_msg_tx_ctxt_lookup(tx_buf->pkt.field.hdr.src_eid,tx_buf->pkt.field.hdr.dst_eid,
                                                    tx_buf->pkt.field.hdr.msg_tag);
                        if (mctp_tx_ctxt != NULL) {
                            msg_type_tx = mctp_tx_ctxt->message_type;
                        }
                    }
                    if ((bool)tx_buf->pkt.field.hdr.eom == true)
                    {
                        mctp_tx_ctxt = mctp_msg_tx_ctxt_lookup(tx_buf->pkt.field.hdr.src_eid,tx_buf->pkt.field.hdr.dst_eid,
                                                    tx_buf->pkt.field.hdr.msg_tag);
                        if (mctp_tx_ctxt != NULL) {
                           mctp_tx_ctxt->in_active_state = false;
                        }
                    }
                    {
                        (void)memcpy(&transmit_buf[0], (uint8_t *)&tx_buf->pkt.data[MCTP_PKT_DST_ADDR_POS], (uint32_t)hdr_struct_size);
                        (void)memcpy(&transmit_buf[hdr_struct_size], (uint8_t *)&tx_buf->pkt.data[10], (uint32_t)(MCTP_PKT_BUF_DATALEN - hdr_struct_size));
                        (void)memcpy(&tx_buf->pkt.data[0], (uint8_t *)&transmit_buf[0], (uint32_t)sizeof(MCTP_BUFDATA));
                    }
                    {
                        /* check timeout condition */
                       if(MCTP_FALSE == (bool)mctp_tx_timeout(tx_buf))
                        {
                            /* change state to transmit it's data over smbus */
                            mctp_tx_state = (uint8_t)MCTP_TX_SMBUS_ACQUIRE;

                            /*Process the non empty buffer*/
                            /*This break comes out of "for" loop*/
                            break;
                        }
                        /* if 135ms timeout */
                        else
                        {
                            /* drop packet, free that buffer */
                            mctp_smbdone_drop(tx_buf);
                            /* for indexing to next tx buffer */
                            mctp_txbuf_index++;

                            /* since tx buffers ranges from MCTP_BUF1 to MCTP_BUF5 */
                            if(mctp_txbuf_index > MCTP_BUF5)
                            {
                                mctp_txbuf_index = MCTP_BUF1;
                            }
                        }
                    }
                }
                /* if other than ec tx response buffer; */
                /* then no need to check 100ms timeout condition */
                else
                {
                    /* change state to transmit it's data over smbus */
                    mctp_tx_state = (uint8_t)MCTP_TX_SMBUS_ACQUIRE;
                    start_time = (uint32_t)(mctp_i2c_get_current_timestamp());
                    // interval = 0; // Coverity security Fixes, variable unused
                    /*Process the non empty buffer*/
                    /*This break comes out of "for" loop*/
                    break;
                }
            }
            else
            {
                /* for indexing to next tx buffer */
                mctp_txbuf_index++;

                /* since tx buffers ranges from MCTP_BUF1 to MCTP_BUF2 */
                if(mctp_txbuf_index > MCTP_BUF5)
                {
                    mctp_txbuf_index = MCTP_BUF1;
                }
            }
        }
        /* if no valid TX buffer is found */
        if(mctp_tx_state == (uint8_t)MCTP_TX_NEXT)
        {
            /* change mctp tx state to idle */
            mctp_tx_state = (uint8_t)MCTP_TX_IDLE;

            /* initialize to start with MCTP_BUF1; */
            mctp_txbuf_index = MCTP_BUF1;

            /*This break ends switch case*/
            break;
        }
    /*If code reaches here, one or more TX buffers are active. So
     * fall-through to next state*/

    case (uint8_t)MCTP_TX_SMBUS_ACQUIRE:
        /*Check if smbus can be acquired. We are not running preemptive
         * kernel. So status check and bus usage following that, are atomic
         * in nature*/
        /* get current TX buffer pointer */
        acquire_status = mctp_i2c_get_chan_busy_status(MCTP_I2C_CHANNEL);
        bool avail = false;
        if((uint8_t)I2C_MASTER_AVAILABLE == acquire_status)
        {
            avail = true;
            mctp_wait_smbus_callback = 0x0;
        }
        if(avail)
        {
            /*smbus is available, start transmission*/
            mctp_tx_state = (uint8_t)MCTP_TX_IN_PROGRESS;
            /*Fall Through to MCTP_TX_IN_PROGRESS state*/
        }             
        else
        {
            interval = mctp_timer_difference(start_time);
            uint32_t tout;
            tout = MCTP_SMBUS_AQ_TIMEOUT;
            if(interval >= tout )//135000 = 135ms         
            {
                /* update buffer parameters and configure events */
                mctp_smbdone_handler(tx_buf);
                mctp_tx_state = (uint8_t)MCTP_TX_NEXT;

            }
            else
            {
                mctp_tx_state = (uint8_t)MCTP_TX_SMBUS_ACQUIRE;
            }

            SET_MCTP_EVENT_TASK(mctp);

            /*Exit switch case*/
            break;
        }
    /*Fall thorugh to the next state MCTP_TX_IN_PROGRESS*/
    case (uint8_t)MCTP_TX_IN_PROGRESS:
        if(0x00U == mctp_wait_smbus_callback)
        {
            mctp_wait_smbus_callback = 1;

            mctp_transmit_smbus(tx_buf);
        }
        break;

    default:
        /* Invalid state */
        break;
    }
} /* End mctp_event_tx_handler() */

/******************************************************************************/
/** mctp_event_rx_handler is called from mctp_event_task for handling
* packet received over smbus.
* @param void
* @return void
*******************************************************************************/
void mctp_event_rx_handler(void)
{
    // EC RX REQUEST HANDLER - checks if EC RX REQUEST pkt is pending, and processes it.
    mctp_handle_ec_rx_request_pkt();

} /* End mctp_event_rx_handler() */

/******************************************************************************/
/** EC RX REQUEST HANDLER - checks if EC RX REQUEST pkt is pending, and processes it.
* @param void
* @return void
*******************************************************************************/
void mctp_handle_ec_rx_request_pkt(void)
{
    MCTP_PKT_BUF *rx_buf;
    MCTP_PKT_BUF *tx_resp_buf;

    /* get pointer of ec rx request buffer */
    rx_buf      = (MCTP_PKT_BUF *) &mctp_pktbuf[MCTP_BUF1];
    /* get pointer of ec tx response buffer */
    tx_resp_buf = (MCTP_PKT_BUF *) &mctp_pktbuf[MCTP_BUF1];

    /* if ec rx request buffer has valid packet */
    if(rx_buf->buf_full == (uint8_t)MCTP_RX_PENDING)
    {
        /* if packet type is control */
        if(rx_buf->pkt.field.hdr.msg_type == MCTP_MSGTYPE_CONTROL)
        {
            /// There will be a message availabe for the control message type eithe Valid or error
            if((0U == mctp_rt.ep.mc.field.current_eid )|| ( 0U == mctp_rt.ep.mc.field.smb_address))
            {
                mctp_rt.ep.mc.field.current_eid = rx_buf->pkt.field.hdr.src_eid;
                mctp_rt.ep.mc.field.smb_address = rx_buf->pkt.field.hdr.src_addr;
            }

            mctp_ec_control_pkt_handler(rx_buf, tx_resp_buf);
            mctp_smbus_txpktready_init(tx_resp_buf);
        }
    }
} /* mctp_handle_ec_rx_request_pkt */

/******************************************************************************/
/** Validates packet received over smbus.
* @param *pktbuf Pointer to smbus layer packet buffer
* @return MCTP_TRUE if packet is found valid, else return MCTP_FALSE
*******************************************************************************/
uint8_t mctp_packet_validation(uint8_t *pkt_buf)
{
    MCTP_IDENTITY *mctp_ctx = NULL;

    //Check this checking
    if (!((pkt_buf[MCTP_PKT_BYTE_CNT_POS] >= MCTP_BYTECNT_MIN) &&
            (pkt_buf[MCTP_PKT_BYTE_CNT_POS] <= MCTP_BYTECNT_MAX)))
    {
        return (uint8_t)MCTP_FALSE;
    }

    if (pkt_buf[MCTP_PKT_HDR_VER_POS] != MCTP_HDR_VER_REF)
    {
        return (uint8_t)MCTP_FALSE;
    }

    if (((pkt_buf[MCTP_PKT_TO_MSGTAG_POS] & MCTP_SOM_EOM_REF_MSK) == MCTP_SOM_EOM_REF))
    {
        if (!((pkt_buf[MCTP_PKT_DST_EID_POS] == mctp_rt.ep.ec.field.current_eid) ||
                (pkt_buf[MCTP_PKT_DST_EID_POS] == MCTP_NULL_EID)))
        {
            return (uint8_t)MCTP_FALSE;
        }
        if (MCTP_OTHER_PKT == mctp_get_packet_type(pkt_buf)
                )
        {
            return (uint8_t)MCTP_FALSE;
        }
        active_pkt_msg_type_rx = pkt_buf[MCTP_PKT_IC_MSGTYPE_POS];
    }

    //if SOM == 1, EOM == 0 || SOM == 0, EOM == 0 || SOM == 0 , EOM == 1
    else if (!((pkt_buf[MCTP_PKT_TO_MSGTAG_POS] & MCTP_SOM_EOM_REF_MSK) == MCTP_SOM_EOM_REF))
    {
        if (!((pkt_buf[MCTP_PKT_DST_EID_POS] == mctp_rt.ep.ec.field.current_eid) ||
                (pkt_buf[MCTP_PKT_DST_EID_POS] == MCTP_NULL_EID)))
        {
            return (uint8_t)MCTP_FALSE;
        }

        if( (pkt_buf[MCTP_PKT_TO_MSGTAG_POS] & MCTP_SOM_REF_MSK) == MCTP_SOM_REF ) // IF SOM ==1
        {
            if (MCTP_OTHER_PKT == mctp_get_packet_type(pkt_buf)
                    )
            {
                return (uint8_t)MCTP_FALSE;
            }
            mctp_ctx = mctp_msg_ctxt_lookup(pkt_buf);

            if (mctp_ctx) {
                mctp_msg_ctxt_reset(mctp_ctx);
                //  If an existing context is present already, don't create new one
            } else {
                mctp_ctx = mctp_msg_ctxt_create(pkt_buf);
                if (!mctp_ctx) {
                    return (uint8_t)MCTP_FALSE;
                }
            }
            mctp_ctx->packet_seq = 0;
            active_pkt_msg_type_rx = mctp_ctx->message_type;
            mctp_ctx->packetizing = true;
        }
        else if ((pkt_buf[MCTP_PKT_TO_MSGTAG_POS] & MCTP_EOM_REF_MSK) == MCTP_EOM_REF) // IF EOM == 1
        {
            mctp_ctx = mctp_msg_ctxt_lookup(pkt_buf);
            if (!mctp_ctx) {
                return (uint8_t)MCTP_FALSE;
            }

            if (((pkt_buf[MCTP_PKT_TO_MSGTAG_POS] & MCTP_MSG_PKSEQ_REF_MASK) >> MCTP_MSG_PKSEQ_SHIFT) !=
                (mctp_ctx->packet_seq + 1) % 4)
            {
                mctp_msg_ctxt_drop(mctp_ctx);
                return (uint8_t)MCTP_FALSE;
            }
            active_pkt_msg_type_rx = mctp_ctx->message_type;
            mctp_ctx->in_active_state = false;
            
        }
        else if ((pkt_buf[MCTP_PKT_TO_MSGTAG_POS] & MCTP_SOM_EOM_REF) == 0x00)
        {
            mctp_ctx = mctp_msg_ctxt_lookup(pkt_buf);
            if (!mctp_ctx) {
                return (uint8_t)MCTP_FALSE;
            }
            
            if (((pkt_buf[MCTP_PKT_TO_MSGTAG_POS] & MCTP_MSG_PKSEQ_REF_MASK) >> MCTP_MSG_PKSEQ_SHIFT) !=
                (mctp_ctx->packet_seq + 1) % 4)
            {
                mctp_msg_ctxt_drop(mctp_ctx);
                return (uint8_t)MCTP_FALSE;
            } else {
                active_pkt_msg_type_rx = mctp_ctx->message_type;
                mctp_ctx->packet_seq = 
                       ((pkt_buf[MCTP_PKT_TO_MSGTAG_POS] & MCTP_MSG_PKSEQ_REF_MASK) >> MCTP_MSG_PKSEQ_SHIFT);
            }
        }
        // mctp_base_packetizing_val_set(true);
    }
    else
    {
        // mctp_base_packetizing_val_set(false);
        return (uint8_t)MCTP_FALSE;
    }

    return (uint8_t)MCTP_TRUE;

} /* End mctp_packet_validation() */

/******************************************************************************/
/** mctp_msg_ctxt_drop
* @param *mctp_ctxt Pointer to mtp ctxt to be dropped
* @return none
*******************************************************************************/
void mctp_msg_ctxt_drop(MCTP_IDENTITY *mctp_ctxt)
{
    if(mctp_ctxt != NULL)
    {
        mctp_ctxt->in_active_state = false;
    }
}

/******************************************************************************/
/** mctp_msg_tx_ctxt_lookup
* @param src_eid
* @param dst_eid
* @param msg_tag
* @return pointer to mctp contest if lookup is success, else NULL pointer
*******************************************************************************/
MCTP_TX_CXT * mctp_msg_tx_ctxt_lookup (uint8_t src_eid, uint8_t dst_eid, uint8_t msg_tag)
{
    uint8_t i;

    for (i=0; i <2 ; i++)
    {
        if (mctp_tx[i].in_active_state &&
            (mctp_tx[i].source_endpt == src_eid) &&
            (mctp_tx[i].destination_endpt == dst_eid) &&
            (mctp_tx[i].message_tag == msg_tag))
        {
            return &mctp_tx[i];
        }
    }
    return NULL;
}
/******************************************************************************/
/** mctp_msg_tx_ctxt_create
* @param msg_type
* @param src_eid
* @param dst_eid
* @param msg_tag
* @return pointer to created context
*******************************************************************************/
MCTP_TX_CXT * mctp_msg_tx_ctxt_create (uint8_t msg_type, uint8_t src_eid, uint8_t dst_eid, uint8_t msg_tag)
{
    uint8_t i;

    for (i=0; i <2 ; i++)
    {
        if (!mctp_tx[i].in_active_state)
        {
            mctp_tx[i].source_endpt = src_eid;
            mctp_tx[i].destination_endpt = dst_eid;
            mctp_tx[i].message_tag = msg_tag;
            mctp_tx[i].message_type = msg_type;
            mctp_tx[i].in_active_state = true;
            return &mctp_tx[i];
        }
    }
    return NULL;
}

/******************************************************************************/
/** mctp_msg_ctxt_reset
* @param *mctp_ctxt Context for which buffer parameters needs to be reset
* @return none
*******************************************************************************/
void mctp_msg_ctxt_reset(MCTP_IDENTITY *mctp_ctxt)
{
    mctp_ctxt->buf_index = 0;
    mctp_ctxt->buf_size = 0;
}
/******************************************************************************/
/** mctp_msg_ctxt_create
* @param *pktbuf Pointer to smbus layer packet buffer
* @return pointer to created mctp msg context
*******************************************************************************/
MCTP_IDENTITY * mctp_msg_ctxt_create(uint8_t *pkt_buf)
{
    uint8_t i;

    for (i=0; i <2 ; i++)
    {
        if (!mctp_rx[i].in_active_state)
        {
            mctp_rx[i].source_endpt = pkt_buf[MCTP_PKT_SRC_EID_POS];
            mctp_rx[i].destination_endpt = pkt_buf[MCTP_PKT_DST_EID_POS];
            mctp_rx[i].message_tag = pkt_buf[MCTP_PKT_TO_MSGTAG_POS] & MCTP_MSG_TAG_REF_MASK;
            mctp_rx[i].tag_owner = pkt_buf[MCTP_PKT_TO_MSGTAG_POS] & MCTP_MSG_TAG_TO_REF_MASK;
            mctp_rx[i].message_type = pkt_buf[MCTP_PKT_IC_MSGTYPE_POS];
            mctp_rx[i].packet_seq = (pkt_buf[MCTP_PKT_TO_MSGTAG_POS] & MCTP_MSG_PKSEQ_REF_MASK) >> MCTP_MSG_PKSEQ_SHIFT;
            mctp_rx[i].in_active_state = true;
            return &mctp_rx[i];
        }
    }
    return NULL;
}
/******************************************************************************/
/** MCTP message lookup for message reassembly
* @param *pktbuf Pointer to smbus layer packet buffer
* @return pointer to mctp contest if lookup is success, else NULL pointer
*******************************************************************************/
MCTP_IDENTITY * mctp_msg_ctxt_lookup(uint8_t *pkt_buf)
{
    uint8_t i;

    for (i=0; i< 2; i++)
    {
        if ((mctp_rx[i].source_endpt == pkt_buf[MCTP_PKT_SRC_EID_POS]) &&
            (mctp_rx[i].destination_endpt == pkt_buf[MCTP_PKT_DST_EID_POS]) &&
            (mctp_rx[i].message_tag == (pkt_buf[MCTP_PKT_TO_MSGTAG_POS] & MCTP_MSG_TAG_REF_MASK)) &&
            (mctp_rx[i].tag_owner == (pkt_buf[MCTP_PKT_TO_MSGTAG_POS] & MCTP_MSG_TAG_TO_REF_MASK)))
        {
            return &mctp_rx[i];
        }
    }

    return NULL;
}

/******************************************************************************/
/** For response TX packet buffer, this compares current time with initial
* reference time (when corresponding request packet was received over smbus)
* to check 135ms timeout condition.
* @param *tx_buf Pointer to TX response packet buffer
* @return MCTP_TRUE if trasmission timeout, else return MCTP_FALSE
*******************************************************************************/
uint8_t mctp_tx_timeout(MCTP_PKT_BUF *tx_buf)
{
    uint16_t processing_time;

    uint16_t max_tick_count = 0x00;

    MCTP_TX_CXT *mctp_tx_ctxt = NULL;

    /* get current request_per_tx_timeout_count */
    processing_time = (uint16_t)mctp_timer_difference(tx_buf->rx_timestamp);

    mctp_tx_ctxt = mctp_msg_tx_ctxt_lookup(tx_buf->pkt.field.hdr.src_eid,tx_buf->pkt.field.hdr.dst_eid,
                                tx_buf->pkt.field.hdr.msg_tag);
    if (mctp_tx_ctxt != NULL) {
        msg_type_tx = mctp_tx_ctxt->message_type;
    }

    if (msg_type_tx == MCTP_MSGTYPE_CONTROL)
    {
        max_tick_count = (uint16_t)((MCTP_TIMEOUT_MS * configTICK_RATE_HZ)/1000UL); //max 100 ms timeout
    }
    /* check for timeout condition */
    if(processing_time <= max_tick_count)
    {
        return (uint8_t)MCTP_FALSE;
    }
    else
    {
        return (uint8_t)MCTP_TRUE;
    }

} /* End mctp_tx_timeout() */

/******************************************************************************/
/** For Calculating the time difference between current and the time given
* @param start_time_val Start time counter value
* @return Return the difference between start and end timing
*******************************************************************************/
uint32_t mctp_timer_difference(uint32_t start_time_val)
{
    uint32_t interval;

    /* get current time interval */
    interval = (uint32_t)(tx_time_get());

    if(interval >= start_time_val)
    {
        interval = (interval - start_time_val);
    }
    else
    {
        interval = ((0xFFFFFFFFU - start_time_val) + interval);
    }

    return interval;
}

/******************************************************************************/
/** Get packet type - response or request or other.
* @param *buffer_ptr Pointer to packet buffer
* @return Packet type - request, response or other.
*******************************************************************************/
uint8_t mctp_get_packet_type(uint8_t *buffer_ptr)
{
    uint8_t tag_owner_field;
    uint8_t req_field;
    uint8_t dtgram_field;

    tag_owner_field = (buffer_ptr[MCTP_PKT_TO_MSGTAG_POS] & MCTP_HDR_MASK_TO) >> 3;
    req_field       = (buffer_ptr[MCTP_PKT_RQ_D_POS] & MCTP_HDR_MASK_RQ) >> 7;
    dtgram_field    = (buffer_ptr[MCTP_PKT_RQ_D_POS] & MCTP_HDR_MASK_D) >> 6;

    /* if request packet */
    if((tag_owner_field == 1U) && (req_field == 1U) && (dtgram_field == 0U))
    {
        return MCTP_REQ_PKT;
    }    /* else if response packet */
    else if((tag_owner_field == 0U) && (req_field == 0U) && (dtgram_field == 0U))
    {
        return MCTP_RESP_PKT;
    }
    /* else neither request nor response */
    else
    {
        return MCTP_OTHER_PKT;
    }

} /* End mctp_get_packet_type */

/****************************************************************/
/** SET_MCTP_TX_STATE
* Set MCTP module to state machine to process tx of next
* available packet
* @param  void
* @return void
/****************************************************************/
void SET_MCTP_TX_STATE(void)
{
    mctp_tx_state = (uint8_t)MCTP_TX_NEXT;
}

uint16_t tx_time_get()
{
    return (uint16_t) (xTaskGetTickCount() / 10);
}

/**   @}
 */

