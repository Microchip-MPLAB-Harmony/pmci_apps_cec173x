/*****************************************************************************
* � 2013 Microchip Technology Inc. and its subsidiaries.
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
/** @defgroup smb smb
 *  @{
 */
/** @file smb.h
 \brief the smb export header file
 This file exports the function prototypes, typedefs, enums that are required by
 smbus applications

<b>Platform:</b> This is ARM-based component
  Details of hardware blocks on which this smbus reuse component is tested:
    smbus HW Block name - blk_i2c_spb, smbus HW Block build # - B0076   
    dma HW Block name - blk_dma_ahb, dma HW Block build # - B0007
    FPGA build used for testing -> MEC1322 FPGA build Rev-81-Randy - 081_chip_top_fpga2_run1
    //depotAE/projects/MEC1322/FPGA builds/Rev-81-Randy/081_chip_top_fpga2_run1.mcs

<b>Toolset:</b> MDK-Lite V 4.23; Armcc, Armasm, ArmLink, ArmAr, FromElf - V4.1.0.894 
<b>Reference:</b> smsc_reusable_fw_requirement.doc */

/*******************************************************************************
 $File: //depot_pcs/FWEng/solutions/Glacier_GEN3/maincodeline/Harmony/pmci_stack/pmci_apps_cec173x/apps/spdm/firmware/src/smbus/smb.h $

Version Control Information (Perforce)
*******************************************************************************
$Revision: #2 $
$DateTime: 2023/02/24 06:07:07 $
$Author: i64652 $
  Change Description:
    2. Code changes to support Master Tx chaining
    1. Branched from //depot_pcs/FWEng/projects/MEC1324/maincodeline/libSmbus/source/smb/ 
        label - MEC1324_LIB_SMBUS_0400
*******************************************************************************/
#ifndef SMB_H_
#define SMB_H_

/* SMBUS driver is configured for freeRTOS environment*/
#define SMB_FREERTOS    1u
#define SMB_SKERN       0u
#include "rtos_definitions.h"


#ifdef  __cplusplus
extern "C" {
#endif
#include <common.h>


// Channel enums
enum smb_channels
{
    SMB_CHANNEL_0 = 0
   ,SMB_CHANNEL_1
   ,SMB_CHANNEL_2
   ,SMB_CHANNEL_3
   ,SMB_CHANNEL_4
};

/* Port Enums */
enum smb_ports
{
    SMB_PORT_0 = 0
   ,SMB_PORT_1
   ,SMB_PORT_2
   ,SMB_PORT_3
   ,SMB_PORT_4
   ,SMB_PORT_5
   ,SMB_PORT_6
   ,SMB_PORT_7
   ,SMB_PORT_8
   ,SMB_PORT_9
   ,SMB_PORT_10 
   ,SMB_PORT_11
   ,SMB_PORT_12
   ,SMB_PORT_13
   ,SMB_PORT_14
   ,SMB_PORT_15
};


/* Speed Enums */
enum speeds
{
    SMBUS_SPEED_100KHZ
   ,SMBUS_SPEED_400KHZ
   ,SMBUS_SPEED_1MHZ
};

typedef UINT8 (*SMB_CALLBACK_FUNC_PTR)(UINT8, UINT8, UINT8);

/** Master export Section */
/*===================================================================
=====================================================================*/

/** Return values for smb_protocol_execute */
#ifndef DISABLE_SMB_MASTER
enum MasterReturnValue
{
    MASTER_ERROR =0 /**< Master is busy, retry after sometime */
    ,MASTER_OK          /**< Master command under execution */
};

enum MasterBusyStatus
{
    MASTER_BUSY =0      /**< Master is busy, retry after sometime */
    ,MASTER_AVAILABLE   /**< Master is available, go ahead with transaction */
};

/******************************************************************************/
/** struct SMB_MAPP_CBK_NEW_TX \n
 * This structure is used to get information for new Tx from Application Callback
*******************************************************************************/
typedef struct SMB_MAPP_CBK_NEW_TX_
{
    UINT8 *buffer_ptr;      /**< Application buffer */
    UINT8 smb_protocol;     /**< SMB Protocol */
    UINT8 WriteCount;       /**< Write Count */
    UINT8 pecEnable;        /**< PEC Enable/Disable Flag */
}SMB_MAPP_CBK_NEW_TX;

typedef UINT8 (*MASTER_FUNC_PTR)(UINT8, UINT8, UINT8 *, SMB_MAPP_CBK_NEW_TX *);

/******************************************************************************/
/** Status codes for master transaction. The error codes are for Bus Error,
 * MNAKX and PEC Error. The success codes are for successful tx and successful
 * rx (we could have only one success code, though)
 *******************************************************************************/
enum MasterStatus
{
    ERROR_BER_TIMEOUT =0        /**< Bus Error due to Timeout */
    ,ERROR_BER_NON_TIMEOUT      /**< Bus Error due to Non Timeout */
    ,ERROR_LAB                  /**< Lost Arbitration Error */
    ,ERROR_MADDR_NAKX           /**< Slave sent Address NACK */
    ,ERROR_MDATA_NAKX           /**< Slave sent Data NACK */
    ,ERROR_SMB_DISABLED         /**< smbus is disabled thru Vreg */
    ,ERROR_CLK_DATA_NOT_HIGH    /**< CLK or Data Not High */
    ,ERROR_PEC                  /**< PEC Error */
    ,SUCCESS_TX                 /**< Successful Master Tx */
    ,SUCCESS_RX                 /**< Successful Master Rx */
    ,SUCCESS_TX_CHAINED       /**< Successful Master Tx for chained transfer - intermediate status*/
    ,SUCCESS_RX_CHAINED       /**< Successful Master Rx for chained transfer - intermediate status*/
};

/* Maximum size of read block master packet */
#define SMB_MAX_MASTER_READ_BLOCK_SIZE  240    //80

#endif //#ifndef DISABLE_SMB_MASTER

/** Application return values */
enum ApplicationReturnValues
{
    APP_RETVAL_RELEASE_SMBUS =0   /**< Application releases hold on smbus */
    ,APP_RETVAL_HOLD_SMBUS        /**< Application still acquires smbus */
    ,APP_RETVAL_RETRY                           /**< Application wants to retry the current transaction */
    ,APP_RETVAL_NEW_TX                      /**< Application wants to start new Master TX immediately */
    ,APP_RETVAL_CHAINED_RX              /**< Application is continuing a chained RX transaction */
    ,APP_RETVAL_CHAINED_RX_LAST     /**< Last request for a chained RX transaction */
    ,APP_RETVAL_CHAINED_TX              /**< Application is continuing a chained TX transaction */
    ,APP_RETVAL_CHAINED_TX_LAST     /**< Last request for a chained TX transaction */
};

/* -------------------- SMB Protocol Type --------------------------------------
   SMB table PROTOCOL TYPE

   Bit 7,6 - 00 = No Operation.
             01 = Write number of bytes in SMB buffer [1].
             10 = Write number of bytes in SMB buffer [1]
                  then read  number of bytes in bits 5 - 0.
             11 = Write number of bytes in SMB buffer [1]
                  then do repeated start condition, then
                  read number of bytes in bits 5 - 0.

   Bit 5 - 0    = Number of read bytes:
                  0 - 3E : read fixed number of bytes from 0 - 3Eh
                  3F     : read number of bytes indicated by first
                           byte of return string from SMB device. */
#define I2C_NOP    0
#define I2C_WRITE  1
#define I2C_READ   2
#define I2C_REPEAT 3

#define SMB_I2C_WRITE                               ((I2C_WRITE  << 6) + 0x00)
#define SMB_I2C_READ                                ((I2C_READ   << 6) + 0x00)
#define SMB_I2C_COMBINED                            ((I2C_REPEAT << 6) + 0x00)

#define SMB_QUICK_COMMAND                           ((I2C_WRITE  << 6) + 0x00)
#define SMB_SEND_BYTE                               ((I2C_WRITE  << 6) + 0x00)
#define SMB_RECEIVE_BYTE                            ((I2C_READ   << 6) + 0x01)
#define SMB_WRITE_BYTE                              ((I2C_WRITE  << 6) + 0x00)
#define SMB_WRITE_WORD                              ((I2C_WRITE  << 6) + 0x00)

#define SMB_READ_BYTE                               ((I2C_REPEAT << 6) + 0x01)
#define SMB_READ_WORD                               ((I2C_REPEAT << 6) + 0x02)
#define SMB_PROCESS_CALL                            ((I2C_REPEAT << 6) + 0x02)
#define SMB_WRITE_BLOCK                             ((I2C_WRITE  << 6) + 0x00)
#define SMB_READ_BLOCK                              ((I2C_REPEAT << 6) + 0x3F)
#define SMB_BLOCK_WRITE_BLOCK_READ_PROCESS_CALL     ((I2C_REPEAT << 6) + 0x3F)

/** Slave export Section */
/*===================================================================
=====================================================================*/

/******************************************************************************/
/** Status codes for slave registration by application. Success condition
 * is STATUS_OK. If the application is already registered the return status
 * is STATUS_ALREADY REGISTERED. If maximum number of application has registered,
 * there is no memory to store more application information. In this case the
 * return status is STATUS_MAX_APP_REGISTERED. STATUS_APP_NOT_REGISTERED is
 * returned during de-registration when application registration information
 * is not found.
 *******************************************************************************/
enum SlaveRegistarationStatus
{
    STATUS_ALREADY_REGISTERED = 0 /**< Application has already registered */
    ,STATUS_MAX_APP_REGISTERED     /**< Maximum number of application registered */
    ,STATUS_APP_NOT_REGISTERED     /**< Application is not registered */
    ,STATUS_OK                      /**< Success status */
};

/** Packet acception status by application */
enum SlavePacketStatus
{
    STATUS_BUFFER_NOT_DONE= 0   /**< Packet not accepted, since application is busy */
    ,STATUS_BUFFER_DONE         /**< Packet accepted by application */
    ,STATUS_BUFFER_ERROR        /**< Packet not meant for this application */
};

enum SlaveTransmitStatus
{
    SLAVE_TRANSMIT_FALSE = FALSE    /**< Slave Transmit phase not required */
    ,SLAVE_TRANSMIT_TRUE= TRUE      /**< Slave Transmit phase required */
};

/******************************************************************************/
/** struct BUFFER_INFO \n
 * This structure is used to store information of a slave receive buffer
 * XmitCount is used along with slaveXmitDoneFlag for chaining slave transmit data 
*******************************************************************************/
typedef struct BUFFER_INFO
{
    UINT8 *     buffer_ptr;     /**< Pointer to buffer memory */
    UINT16      TimeStamp;      /**< Packet received timestamp */
    UINT8       DataLen;        /**< Data Length of packet received */
    UINT8       XmitCount;       /**< Number of times slave has transmitted using this buffer for this transaction */
    UINT8       RxCount;       /**< Number of times slave has received using this buffer for this transaction */
    UINT8        pecFlag;        /**< PEC valid/invalid flag */      
    UINT8        slaveXmitDoneFlag;   /**< Flag indicating if xmit is completed by slave application */
    UINT8       channel;        /**< Channel on which this packet is received */
    BOOL        sdoneFlag;      /**< Flag to indicate if SDONE occured for this buffer */  
}BUFFER_INFO;

/******************************************************************************/
/** Slave application function pointer \n
 * The first argument is pointer to BUFFER_INFO structure which will contain
 * details of the packet received. The second parameter is flag to indicate
 * slave transmit phase. In case of slave transmit phase, the application
 * should provide the data to be transmitted in the same buffer and indicate
 * whether PEC should be enabled for transmit phase.
*******************************************************************************/
typedef UINT8 (*SLAVE_FUNC_PTR)(BUFFER_INFO *, UINT8);


/** Callbacks & Events export Section */
/*===================================================================
=====================================================================*/

#define DEFAULT_PORT_MASK           0x0F
#define IGNORE_EVENT_VALUE      0x00u

/** States for smb config enable */
enum smbCfgEnableStates
{
    CFG_ENABLE_NO_CHANGE =0 /**< No change in cfg enable bit */
    ,CFG_ENABLE_0_TO_1          /**< cfg enable bit changed from 0 to 1 */
    ,CFG_ENABLE_1_TO_0          /**< cfg enable bit changed from 1 to 0 */
};

/** smb callback events */
enum smbCallbackEvents
{
     SMB_CBK_DISABLED =0            /**< SMB Disabled */
    ,SMB_CBK_HW_ENABLED             /**< SMB HW Enabled */
    ,SMB_CBK_BER                    /**< SMB Bus Error */
    ,SMB_CBK_BUSY                   /**< SMB Busy */
    ,SMB_CBK_PORT_ERROR_SET             /**< Port Error */
    ,SMB_CBK_PORT_ERROR_CLR             /**< Port Error Clear*/
};

/* port configuration bit definitions */
enum port_config_bits
{
    sbit_PORT_CONFG_DYNAMIC_SWTCH           = BIT_7_MASK
};

/** Slave Nack Mechanism */
enum Slave_OutOfBuffer_Mechanism
{
    SLAVE_LAST_BUFFER_CLK_STRETCH=0     /**< Slave mechanism for clock stretch */
    ,SLAVE_NO_BUFFER_NACK                   /**< Slave mechanism for NACK on 2nd byte */
};

#ifndef DISABLE_SMB_MASTER

/* APIs to check busy status */
extern UINT8 smb_busyStatus_get(const UINT8 channel);
extern UINT8 smb_portBusyStatus_get(const UINT8 channel,const UINT8 port);

/* APIs to change speed & port before starting any master transaction */
extern UINT8 smb_change_port(const UINT8 channel,const UINT8 port);
extern void smb_set_speed(const UINT8 channel,const UINT8 speed);

/* APIs to start master transaction */
extern UINT8 smb_protocol_execute(const UINT8 channel, UINT8 *buffer_ptr,const UINT8 smb_protocol, const UINT8 writeCount,
            const UINT8 pecEnable, MASTER_FUNC_PTR func_ptr, const UINT8 readChainedFlag, const UINT8 writeChainedFlag);
extern UINT8 smb_protocol_execute_blocking(const UINT8 channel, UINT8 *buffer_ptr,const UINT8 smb_protocol
        ,const UINT8 writeCount,const UINT8 pecEnable, MASTER_FUNC_PTR func_ptr, const UINT8 readChainedFlag, const UINT8 writeChainedFlag);
#endif

/* Smbus controller enabling/disabling & configuration */
extern void smbus_configure_and_enable(UINT8 channel, UINT16 own_address, UINT8 speed, UINT8 port, UINT8 configFlag);
extern void smb_enable_timeouts(const UINT8 channel, const UINT8 timeoutsFlag);
extern void smbus_disable(UINT8 channel);

#ifndef DISABLE_ALL_SLAVE_BUFFERS
/* Slave-Application Registration APIs */
extern UINT8 smb_register_slave(const UINT8 channel, SLAVE_FUNC_PTR slaveFuncPtr);
extern UINT8 smb_deregister_slave(const UINT8 channel, SLAVE_FUNC_PTR slaveFuncPtr);
#endif

/*Initializing API */
extern void smbus_init_task(void);

uint32_t smb_task_wait_event_bits(uint32_t event_bits, uint32_t wait_all_bits);

/* To be hooked-in with Application & ThreadX */
#if SMB_FREERTOS
extern void smbus_app_timer(TimerHandle_t pxTimer) ; 
#elif SMB_SKERN
extern void smbus_app_timer(void);
#else
#endif 
extern void smb_isr(void);
extern void smb_dma_isr(void);

extern void smb_raise_interrupt_event(void);
extern void smb_raise_timer_event(const UINT8 timeInMs);
extern UINT16 smb_get_current_timestamp(void);


VOID smb_callback(const UINT8 channel,const UINT8 eventType,const UINT8 eventValue);

#ifdef  __cplusplus
};
#endif

#endif /* SMB_H_ */
/** @}
*/
