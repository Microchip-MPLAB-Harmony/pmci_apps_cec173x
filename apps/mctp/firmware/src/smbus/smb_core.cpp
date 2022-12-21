/*****************************************************************************
* © 2013 Microchip Technology Inc. and its subsidiaries.
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
/** @defgroup smb smb_core
 *  @{
 */
/** @file smb_core.cpp
 \brief the smb core cpp file
This file implements the core smb functionality. It instantiates the
base, master and slave objects for the functioning of smbus driver

<b>Platform:</b> This is ARM-based component
  Details of hardware blocks on which this smbus reuse component is tested:
    smbus HW Block name - blk_i2c_spb, smbus HW Block build # - B0076   
    dma HW Block name - blk_dma_ahb, dma HW Block build # - B0007
    FPGA build used for testing -> MEC1322 FPGA build Rev-81-Randy - 081_chip_top_fpga2_run1
    //depotAE/projects/MEC1322/FPGA builds/Rev-81-Randy/081_chip_top_fpga2_run1.mcs

<b>Toolset:</b> MDK-Lite V 4.23; Armcc, Armasm, ArmLink, ArmAr, FromElf - V4.1.0.894 
<b>Reference:</b> smsc_reusable_fw_requirement.doc */

/*******************************************************************************
 $File: //depot_pcs/FWEng/solutions/Glacier_GEN3/maincodeline/Harmony/secureboot_app_mpu/firmware/src/smbus/smb_core.cpp $

Version Control Information (Perforce)
*******************************************************************************
$Revision: #5 $
$DateTime: 2022/09/21 04:11:31 $
$Author: i64652 $
  Change Description:
    2. Code changes to support Master Tx chaining
    1. Branched from //depot_pcs/FWEng/projects/MEC1324/maincodeline/libSmbus/source/smb/ 
        label - MEC1324_LIB_SMBUS_0400
*******************************************************************************/

extern "C"
{
    #include <common.h>
    #include "interrupt/interrupt_api.h"
}

#include "smb_config.h"
#include "smb.h"
#include "smb_slave.hpp"
#include "smb_master.hpp"
#include "smb_base.hpp"
#include "smb_core.h"

extern UINT32 smbTimerTick;
extern SMB_CALLBACK_FUNC_PTR pSmbCallback;

using namespace SMB;

#if MAX_SMB > 5
#error No support yet for more than 5 smbus controllers
#endif

// Moved to smb_buffers.c
extern "C" UINT8 bufferPool1[];
#if MAX_SMB > 1
extern "C" UINT8 bufferPool2[];
#endif
#if MAX_SMB > 2
extern "C" UINT8 bufferPool3[];
#endif
#if MAX_SMB > 3
extern "C" UINT8 bufferPool4[];
#endif
#if MAX_SMB > 4
extern "C" UINT8 bufferPool5[];
#endif

//HW_DMA_MAIN& dma_main_block =
//    *reinterpret_cast<HW_DMA_MAIN *> (HW_DMA_BLOCK_MAIN);

#ifndef DISABLE_SMB_MASTER
SMB_BASE base[MAX_SMB];
#else // ifndef DISABLE_SMB_MASTER
SMB_BASE base[MAX_SMB];
#endif // ifndef DISABLE_SMB_MASTER

SMB_SLAVE slave[MAX_SMB];

#ifndef DISABLE_SMB_MASTER
SMB_MASTER master[MAX_SMB];

/* array of master callback function pointers to support blocking requests */
MASTER_FUNC_PTR master_func[MAX_SMB];
#endif //#ifndef DISABLE_SMB_MASTER

void enable_smb_irq(const UINT8 channel);
void enable_smb_dma_irq(const UINT8 channel);
void disable_smb_irq(const UINT8 channel);
bool handle_smb_disable(const UINT8 channel);
UINT8 smb_get_ber_counter_value(const UINT8 channel);
#ifdef HANDLE_POWER_LOW_TO_HIGH_EVENT
void handle_power_low_to_high_event(const UINT8 channel);
#endif

UINT8* get_slave_buffer(const UINT8 channel, UINT8 *pBuffer_size)
{
    UINT8 *buffer_ptr;
    
    buffer_ptr = (UINT8 *)((void *)0);
    if (SMB_CHANNEL_0 == channel)
    {
        buffer_ptr = (UINT8 *)(&bufferPool1);
        *pBuffer_size = SLAVE1_BUFFER_SIZE;
    }
    #if MAX_SMB > 1
    if (SMB_CHANNEL_1 == channel)
    {
        buffer_ptr = (UINT8 *)(&bufferPool2);
    *pBuffer_size = SLAVE2_BUFFER_SIZE;     
    }   
    #endif
    #if MAX_SMB > 2
    if (SMB_CHANNEL_2 == channel)
    {
        buffer_ptr = (UINT8 *)(&bufferPool3);
    *pBuffer_size = SLAVE3_BUFFER_SIZE;     
    }   
    #endif
    #if MAX_SMB > 3
    if (SMB_CHANNEL_3 == channel)
    {
        buffer_ptr = (UINT8 *)(&bufferPool4);
    *pBuffer_size = SLAVE4_BUFFER_SIZE;     
    }   
    #endif
    #if MAX_SMB > 4
    if (SMB_CHANNEL_4 == channel)
    {
        buffer_ptr = (UINT8 *)(&bufferPool5);
    *pBuffer_size = SLAVE5_BUFFER_SIZE;     
    }   
    #endif    
    return buffer_ptr;
    
}

/******************************************************************************/
/** SMBus core init task function
 * This function initializes the smbus base, master and slave
 * @return None
 * @param None
 ******************************************************************************/
void smbus_core_init_task(void)
{
    UINT8 channel;
    UINT8 *buffer_ptr;
    UINT8 buffer_size;

    //trace0(0, SMB_CORE, 0, "smbus_core_init_task: Enter");

    /* For each channel */
    for (channel=0;channel<MAX_SMB;channel++)
    {
#ifndef DISABLE_SMB_MASTER
        master[channel].init(channel);
        master_func[channel] = (MASTER_FUNC_PTR)NULLVOIDPTR;
#endif
        buffer_ptr = get_slave_buffer(channel, &buffer_size);
        slave[channel].init(channel, buffer_ptr, buffer_size);
        base[channel].init(channel);
                
        disable_smb_irq(channel);
        
    }
    DMA_Enable();

}/* End smbus_core_init_task() */

/******************************************************************************/
/** smbus_interrupt_task function
 * This function handles interrupt task functionality of smbus driver
 * @return None
 * @param None
 ******************************************************************************/
void smbus_interrupt_task(void)
{
    UINT8 channel;
#ifndef DISABLE_PENDING_PACKETS 
    bool eventTaskReqFlag;
#endif  

    for (channel=0; channel<MAX_SMB; channel++)
    {
        trace1(0, SMB_CORE, 0, "smbus_interrupt_task: channel %02Xh", channel);
#ifndef DISABLE_SMB_MASTER
        master[channel].interrupt_task();
#endif
        /* check if smb needs to disabled and process it */
        if ( handle_smb_disable(channel))
        {
            /* If smb is disabled, no need for further processing, so switch to next controller */
            continue;
        }
        if ( base[channel].is_bus_error())
        {
            if (pSmbCallback)
            {
                pSmbCallback(channel, SMB_CBK_BER, base[channel].get_ber_counter_value());
            }
#ifndef DISABLE_SMB_MASTER
            master[channel].handle_bus_error();
#endif
            /* check if smb needs to disabled and process it */
            if ( handle_smb_disable(channel))
            {
                /* If smb is disabled, no need for further processing, so switch to next controller */
                continue;
            }
            slave[channel].handle_bus_error();
            base[channel].handle_bus_error();
            slave[channel].enable();
            base[channel].handle_bus_error_postprocessing_ports();
            /* schedule timer task for executing port state machine */
#if SMB_FREERTOS
			smb_raise_timer_event(SMBUS_TIMER_TICK_MSEC);

#elif SMB_SKERN
            kSET_EVENT_WAKETIMER(ms_to_ticks(SMBUS_TIMER_TICK_MSEC), smbus);
            smbTimerTick = ms_to_ticks(SMBUS_TIMER_TICK_MSEC);
#else
#endif
        }
#ifdef HANDLE_POWER_LOW_TO_HIGH_EVENT       
        handle_power_low_to_high_event(channel);
#endif      
        
#ifndef DISABLE_ALL_SLAVE_BUFFERS
    #ifndef DISABLE_PENDING_PACKETS
        eventTaskReqFlag = slave[channel].interrupt_task();
        if (eventTaskReqFlag)
        {
            kSET_EVENT_TASK(smbus);
        }
    #else
        slave[channel].interrupt_task();
    #endif
#endif
        
#ifndef DISABLE_SMB_MASTER
        master[channel].retry();
#endif
    } /* for loop */
    return;
} /* smbus_interrupt_task */

/******************************************************************************/
/** SMBus timer task function
 * This function handles timer task functionality of smbus driver
 * @return None
 * @param None
 ******************************************************************************/
void smbus_timer_task(void)
{
    UINT8 channel;
    bool timerEventRequired = false;

    //trace0(0, SMB_CORE, 0, "smbus_timer_task: Enter");

    /* For each channel */
    for (channel=0;channel<MAX_SMB;channel++)
    {
        if ( base[channel].timer_task())
        {
            timerEventRequired = true;
        }
    }

    if (timerEventRequired)
    {
#if SMB_FREERTOS
		smb_raise_timer_event(SMBUS_TIMER_TICK_MSEC);
        
#elif SMB_SKERN
        kSET_EVENT_WAKETIMER(ms_to_ticks(SMBUS_TIMER_TICK_MSEC), smbus);
        smbTimerTick = ms_to_ticks(SMBUS_TIMER_TICK_MSEC);
#else
#endif
    }
}/* smbus_timer_task */

#ifndef DISABLE_ALL_SLAVE_BUFFERS
#ifndef DISABLE_PENDING_PACKETS
/******************************************************************************/
/** smbus_event_task function
 * This function handles event task functionality of smbus driver
 * @return None
 * @param None
 ******************************************************************************/
void smbus_event_task(void)
{
    UINT8 channel;
    bool eventTaskReqFlag;

    //trace0(0, SMB_CORE, 0, "smbus_event_task: Enter");

    eventTaskReqFlag = false;
    /* For each channel */
    for (channel=0;channel<MAX_SMB;channel++)
    {
        /* Currently only slave requires event task notification */
        if (slave[channel].event_task())
        {
            //flag to indicate if further event task processing is required
            eventTaskReqFlag = true;            
        }       
    }
    
    if (eventTaskReqFlag)
    {
        kSET_EVENT_TASK(smbus);
    }
    
}/* smbus_event_task */
#endif
#endif

/******************************************************************************/
/** smb_disable function
 * This function disables the master, slave and base
 * @return None
 * @param channel smb channel
 ******************************************************************************/
void smb_disable(const UINT8 channel)
{
#ifndef DISABLE_PENDING_PACKETS 
    bool eventTaskReqFlag;  
    eventTaskReqFlag = slave[channel].disable();
    if (eventTaskReqFlag)
    {
        kSET_EVENT_TASK(smbus);     
    }
#else
  slave[channel].disable();
#endif  
    
#ifndef DISABLE_SMB_MASTER
    master[channel].disable();
#endif
    
    base[channel].disable();
    disable_smb_irq(channel);
    if (pSmbCallback)
    {
        pSmbCallback(channel, SMB_CBK_DISABLED, IGNORE_EVENT_VALUE);
    }
}/* smb_disable */

/******************************************************************************/
/** smb_enable function
 * This function enables the master, slave and base 
 * @param channel smb channel
 * @return None
 ******************************************************************************/
void smb_enable(const UINT8 channel)
{
    base[channel].enable();
    if (pSmbCallback)
    {
        pSmbCallback(channel, SMB_CBK_HW_ENABLED, IGNORE_EVENT_VALUE);
    }
    enable_smb_irq(channel);    
    enable_smb_dma_irq(channel);
    slave[channel].enable();
#ifndef DISABLE_SMB_MASTER
    master[channel].enable();
#endif
    base[channel].enable_port();
    /* schedule timer task for executing port state machine */
#if SMB_FREERTOS
	smb_raise_timer_event(SMBUS_TIMER_TICK_MSEC);
#elif SMB_SKERN
    kSET_EVENT_WAKETIMER(ms_to_ticks(SMBUS_TIMER_TICK_MSEC), smbus);
    smbTimerTick = ms_to_ticks(SMBUS_TIMER_TICK_MSEC);
#else
#endif
}/* smb_enable */


/******************************************************************************/
/**     smb_core_check_bus_error
*       This function handles bus error
* @param channel smb channel
* @return True/False - True if bus error detected
*******************************************************************************/
bool smb_core_check_bus_error(const UINT8 channel)
{   
    bool retVal;
        
    retVal = false; 
            
    if (base[channel].does_bus_error_needs_processing())
    {
        trace0(0, SMB_CORE, 0, "smb_core_check_bus_error: BER: ");
        base[channel].ber_isr();
#ifndef DISABLE_SMB_MASTER
        master[channel].ber_isr();
#endif
        /*Note: Stopping of slave dma and clearing SDONE is handled
         * in base[channel].ber_isr() function */   
        
        retVal = true;      
    }
    
    return retVal;
    
}

/******************************************************************************/
/**     smb_core_isr_dma
*       This function handles smbus dma isr for each channel
* @param channel smb channel
* @return None
*******************************************************************************/
void smb_core_isr_dma(const UINT8 channel, const bool slave_dma_flag)
{
    
    //trace0(0, SMB_CORE, 0, "smb_core_isr_dma: Enter ");
    if (!(smb_core_check_bus_error(channel)))
    {           
        if (slave_dma_flag)
        {
            slave[channel].dma_isr();   
        }
        else
        {
            master[channel].dma_isr();          
        }
    }

#if SMB_FREERTOS
	smb_raise_interrupt_event();
#elif SMB_SKERN
    kSET_EVENT_INTERRUPT(smbus);    
#else
#endif        
    //trace0(0, SMB_CORE, 0, "smb_core_isr_dma: Leave ");
}

/******************************************************************************/
/**     smb_isr
*       This function handles smbus isr for each channel
* @param channel smb channel
* @return None
*******************************************************************************/
void smb_core_isr(const UINT8 channel)
{   

    bool raiseInterruptTaskRequestFlag;
    
    //trace0(0, SMB_CORE, 0, "smb_core_isr: Enter ");   
    raiseInterruptTaskRequestFlag = false;
            
    if (!(smb_core_check_bus_error(channel)))   
    {
        if ( base[channel].does_mdone_needs_processing())
        {
            trace0(0, SMB_CORE, 0, "smb_core_isr: MDONE: ");
    #ifndef DISABLE_SMB_MASTER          
            if (master[channel].isr())
            {   
                raiseInterruptTaskRequestFlag = true;
            }           
    #else
            //trace0(0, SMB_CORE, 0, "smb_core_isr: ERROR: MDONE should not be set");
    #endif
        } // if ( base[channel].does_mdone_needs_processing())

        if ( base[channel].does_sdone_needs_processing())
        {
            trace0(0, SMB_CORE, 0, "smb_core_isr: SDONE: ");
            if ( slave[channel].isr())
            {                           
                raiseInterruptTaskRequestFlag = true;
            }           
        } // if ( base[channel].does_sdone_needs_processing())      
    }
    else // else of - if (!(smb_core_check_bus_error(channel)))
    {
        raiseInterruptTaskRequestFlag = true;       
    }
    
    if (raiseInterruptTaskRequestFlag)
    {
#if SMB_FREERTOS
		smb_raise_interrupt_event();
#elif SMB_SKERN
        kSET_EVENT_INTERRUPT(smbus);
#else
#endif
    }
    
    //trace0(0, SMB_CORE, 0, "smb_core_isr: Leave ");
}/* smb_core_isr */

/******************************************************************************/
/** smb_core_master_wdt function
 * This function checks if master wdt is expired and handles it accordingly 
 * @param channel smb channel
 * @return None
 ******************************************************************************/
#ifndef DISABLE_SMB_MASTER
void smb_core_master_wdt(const UINT8 channel)
{    
    if ( master[channel].is_wdt_expired())
    {
        base[channel].handle_master_wdt();

        master[channel].set_completion_status(ERROR_BER_NON_TIMEOUT);
        master[channel].handle_bus_error();
        if ( handle_smb_disable(channel))
        {
            /* If smb is disabled, no need for further processing */
            return;
        }
        slave[channel].handle_bus_error();
#ifndef DISABLE_ALL_SLAVE_BUFFERS
        #ifndef DISABLE_PENDING_PACKETS
        if (slave[channel].check_and_process_recvd_pkts())
        {

            kSET_EVENT_TASK(smbus);
        }
        #else
        slave[channel].check_and_process_recvd_pkts();
        #endif
#endif
        base[channel].handle_bus_error();
        slave[channel].enable();
        base[channel].handle_bus_error_postprocessing_ports();
        /* schedule timer task for executing port state machine */
#if SMB_FREERTOS
		smb_raise_timer_event(SMBUS_TIMER_TICK_MSEC);
#elif SMB_SKERN
        kSET_EVENT_WAKETIMER(ms_to_ticks(SMBUS_TIMER_TICK_MSEC), smbus);
        smbTimerTick = ms_to_ticks(SMBUS_TIMER_TICK_MSEC);
#else 
#endif
    }
}/* smb_core_master_wdt */

/******************************************************************************/
/** check_master_availablity function
 * This function checks if master resource is available
 * @param channel smb channel
 * @param defaultPortFlag flag to indicate if default port needs to be checked
 * @param port smb port
 * @return MASTER_AVAILABLE/MASTER_BUSY
 ******************************************************************************/
UINT8 check_master_availablity(const UINT8 channel, const bool defaultPortFlag,
        UINT8 port)
{
    UINT8 retVal;

    if (defaultPortFlag)
    {
        port = base[channel].get_default_port();
    }

    if (MASTER_BUSY == master[channel].check_busy())
    {
        retVal = (UINT8)MASTER_BUSY;
    }
    else if (PORT_IDLE != base[channel].get_port_state(port))
    {
        retVal = (UINT8)MASTER_BUSY;
    }
    else
    {
        retVal = (UINT8)MASTER_AVAILABLE;
    }

    return retVal;
}/* check_master_availablity */

/******************************************************************************/
/** change_port function
 * This function changes the controller port
 * @param channel smb channel
 * @param channel the port to configure
 * @return MASTER_ERROR/MASTER_OK
 ******************************************************************************/
UINT8 change_port(const UINT8 channel,const UINT8 port)
{

    if (MASTER_BUSY == master[channel].is_ready_for_port_change())
    {
        return MASTER_ERROR;
    }

    if (PORT_IDLE != base[channel].get_port_state(port))
    {
        return MASTER_ERROR;
    }

    base[channel].change_port(port);

    return MASTER_OK;
}/* change_port */

/******************************************************************************/
/**     SMB Core Master Callback Function
 *      This function is called once the master transaction is complete
 *       This function is used to support smbus blocking requests
* @param channel channel on which the transaction completed
* @param status status returned from smbus layer
* @param buffer_ptr pointer to buffer
* @param newTxParams details of new transaction to initiate, if any
* @return APP_RETVAL_RELEASE_SMBUS for releasing smbus
*******************************************************************************/
UINT8 smb_core_master_callback(UINT8 channel, UINT8 status, UINT8 *buffer_ptr, SMB_MAPP_CBK_NEW_TX *newTxParams)
{
    
    UINT8 retVal =(UINT8)APP_RETVAL_RELEASE_SMBUS;
    
    /* Inform the application */
    if ((MASTER_FUNC_PTR)NULLVOIDPTR != master_func[channel])   
    {       
        retVal = master_func[channel](channel, status, buffer_ptr, newTxParams);                        
    }   
    
    if (APP_RETVAL_RELEASE_SMBUS == retVal)
    {
        master_func[channel] = (MASTER_FUNC_PTR)NULLVOIDPTR;    
        
    }
    
    return retVal;
    
}

/******************************************************************************/
/**   execute_master_request function
*  Initiates smbus master operation
* @param channel        channel
* @param buffer_ptr     Buffer for the smbus transaction
* @param smb_protocol   smbus protocol byte
* @param pecEnable      Flag to enable/disable PEC
* @param writeCount     Number of bytes to transmit
* @param func_ptr       Function to call after success/failure of the transaction
* @param blockingFlag   flag to indicate if transaction is blocking or not
* @param readChainedFlag     flag to indicate if read needs to be done using dma chaining
* @param writeChainedFlag    flag to indicate if write needs to be done using dma chaining
* @return               TRUE on success, FALSE on error
*******************************************************************************/
bool execute_master_request(const UINT8 channel, UINT8 *buffer_ptr, const UINT8 smb_protocol
                            ,const UINT8 writeCount,const bool pecEnable, MASTER_FUNC_PTR func_ptr, 
                            const bool blockingFlag, const bool readChainedFlag, const bool writeChainedFlag)
{
    if (channel >= MAX_SMB)
    {
        //trace0(0, SMB_CORE, 0, "execute_master_request: Invalid channel");
        return false;
    }

    if (MASTER_BUSY == master[channel].check_busy(func_ptr))
    {
        //TRACE0(41, SMB_CORE, 0, "execute_master_request: master not available");
        return false;
    }

    if (blockingFlag)
    {
        //trace0(0, SMB_CORE, 0, "execute_master_request: Not supporting blocking call for this version of driver");
        return false;
    }

    master[channel].setup(buffer_ptr, pecEnable, func_ptr);
    //buffer_ptr[writeCount] will contain the read count for i2c read
    master[channel].form_cmd_value(smb_protocol, writeCount, buffer_ptr[writeCount], readChainedFlag, writeChainedFlag);
    master[channel].xmit();

    return true;

}/* End execute_master_request() */

#endif //#ifndef DISABLE_SMB_MASTER

#ifndef DISABLE_ALL_SLAVE_BUFFERS
/******************************************************************************/
/** smb_core_register_slave function.
 * This function registers a smbus slave application
 * @channel the channel on which the slave is registered
 * @param slaveFuncPtr   The application function to call on receiving a packet
 * @return  STATUS_OK on successful registration, else error status
*******************************************************************************/
UINT8 smb_core_register_slave(const UINT8 channel, SLAVE_FUNC_PTR slaveFuncPtr)
{
    return slave[channel].register_app(slaveFuncPtr);
}/* smb_core_register_slave */

/******************************************************************************/
/** smb_core_deregister_slave function.
 * This function is used de-register a smbus slave application
 * @channel the channel on which the slave was registered
 * @param   slaveFuncPtr   Application function pointer
 * @return  STATUS_OK on successful de-registration, else error status
*******************************************************************************/
UINT8 smb_core_deregister_slave(const UINT8 channel, SLAVE_FUNC_PTR slaveFuncPtr)
{
    return slave[channel].deregister_app(slaveFuncPtr);
}/* smb_core_register_slave */
#endif //#ifndef DISABLE_ALL_SLAVE_BUFFERS

/******************************************************************************/
/** is_bus_error_flag_set function
 * This function checks if bus error flag is set
 * SMB_MASTER class
 * @param channel smb channel
 * @return TRUE/FALSE
 * @note This function is used by any layer which wants to check if bus error
 * flag is set but doesn't have acess to base object
 ******************************************************************************/
bool is_bus_error_flag_set(const UINT8 channel)
{
    return base[channel].is_bus_error();
}/* is_bus_error_flag_set */

/******************************************************************************/
/**     enable_irq \n\n
*       This function enables the main interrupt irq for smbus controller
* @param channel smb channel
* @return None
*******************************************************************************/
void enable_smb_irq(const UINT8 channel)
{
    /* Enable smbus interrupt */
	switch(channel)
	{
		case SMB_CHANNEL_0:
			interrupt_device_ecia_source_clear(ECIA_AGG_INT_SRC_I2CSMB0);
			interrupt_device_ecia_enable_set(ECIA_AGG_INT_SRC_I2CSMB0);
			break;
		case SMB_CHANNEL_1:
			interrupt_device_ecia_source_clear(ECIA_AGG_INT_SRC_I2CSMB1);
			interrupt_device_ecia_enable_set(ECIA_AGG_INT_SRC_I2CSMB1);
			break;
		case SMB_CHANNEL_2:
			interrupt_device_ecia_source_clear(ECIA_AGG_INT_SRC_I2CSMB2);
			interrupt_device_ecia_enable_set(ECIA_AGG_INT_SRC_I2CSMB2);
			break;
		case SMB_CHANNEL_3:
			interrupt_device_ecia_source_clear(ECIA_AGG_INT_SRC_I2CSMB3);
			interrupt_device_ecia_enable_set(ECIA_AGG_INT_SRC_I2CSMB3);
			break;
		case SMB_CHANNEL_4:
			interrupt_device_ecia_source_clear(ECIA_AGG_INT_SRC_I2CSMB4);
			interrupt_device_ecia_enable_set(ECIA_AGG_INT_SRC_I2CSMB4);
			break;
		default:
			break;
	}

}/* enable_smb_irq */

/******************************************************************************/
/**     enable_smb_slave_dma_irq \n\n
*       This function enables the dma interrupt irq for the slave smbus controller
* @param channel smb channel
* @return None
*******************************************************************************/
void enable_smb_dma_irq(const UINT8 channel)
{
	switch(channel)
	{
		case SMB_CHANNEL_0:
			/* slave dma's start from odd channel numbers 1, 3, 5, 7, 9 */
			interrupt_device_ecia_source_clear(ECIA_AGG_INT_SRC_DMA_CH01);
			interrupt_device_ecia_enable_set(ECIA_AGG_INT_SRC_DMA_CH01);

			/* master dma's start from even channel numbers 0, 2, 4, 6, 8 */
			interrupt_device_ecia_source_clear(ECIA_AGG_INT_SRC_DMA_CH00);
			interrupt_device_ecia_enable_set(ECIA_AGG_INT_SRC_DMA_CH00);
			break;
		case SMB_CHANNEL_1:
			/* slave dma's start from odd channel numbers 1, 3, 5, 7, 9 */
			interrupt_device_ecia_source_clear(ECIA_AGG_INT_SRC_DMA_CH03);
			interrupt_device_ecia_enable_set(ECIA_AGG_INT_SRC_DMA_CH03);

			/* master dma's start from even channel numbers 0, 2, 4, 6, 8 */
			interrupt_device_ecia_source_clear(ECIA_AGG_INT_SRC_DMA_CH02);
			interrupt_device_ecia_enable_set(ECIA_AGG_INT_SRC_DMA_CH02);
			break;
		case SMB_CHANNEL_2:
			/* slave dma's start from odd channel numbers 1, 3, 5, 7, 9 */
			interrupt_device_ecia_source_clear(ECIA_AGG_INT_SRC_DMA_CH05);
			interrupt_device_ecia_enable_set(ECIA_AGG_INT_SRC_DMA_CH05);

			/* master dma's start from even channel numbers 0, 2, 4, 6, 8 */
			interrupt_device_ecia_source_clear(ECIA_AGG_INT_SRC_DMA_CH04);
			interrupt_device_ecia_enable_set(ECIA_AGG_INT_SRC_DMA_CH04);
			break;
		case SMB_CHANNEL_3:
			/* slave dma's start from odd channel numbers 1, 3, 5, 7, 9 */
			interrupt_device_ecia_source_clear(ECIA_AGG_INT_SRC_DMA_CH07);
			interrupt_device_ecia_enable_set(ECIA_AGG_INT_SRC_DMA_CH07);

			/* master dma's start from even channel numbers 0, 2, 4, 6, 8 */
			interrupt_device_ecia_source_clear(ECIA_AGG_INT_SRC_DMA_CH06);
			interrupt_device_ecia_enable_set(ECIA_AGG_INT_SRC_DMA_CH06);
			break;
		case SMB_CHANNEL_4:
			/* slave dma's start from odd channel numbers 1, 3, 5, 7, 9 */
			interrupt_device_ecia_source_clear(ECIA_AGG_INT_SRC_DMA_CH09);
			interrupt_device_ecia_enable_set(ECIA_AGG_INT_SRC_DMA_CH09);

			/* master dma's start from even channel numbers 0, 2, 4, 6, 8 */
			interrupt_device_ecia_source_clear(ECIA_AGG_INT_SRC_DMA_CH08);
			interrupt_device_ecia_enable_set(ECIA_AGG_INT_SRC_DMA_CH08);
			break;
		default:
			break;
	}



}/* enable_smb_slave_dma_irq */

/******************************************************************************/
/**     disable_irq
*       This function disables the main interrupt irq for smbus controller
* @param channel smb channel
* @return None
*******************************************************************************/
void disable_smb_irq(const UINT8 channel)
{
	switch(channel)
	{
		case SMB_CHANNEL_0:
			interrupt_device_ecia_enable_clear(ECIA_AGG_INT_SRC_I2CSMB0);
			break;
		case SMB_CHANNEL_1:
			interrupt_device_ecia_enable_clear(ECIA_AGG_INT_SRC_I2CSMB1);
			break;
		case SMB_CHANNEL_2:
			interrupt_device_ecia_enable_clear(ECIA_AGG_INT_SRC_I2CSMB2);
			break;
		case SMB_CHANNEL_3:
			interrupt_device_ecia_enable_clear(ECIA_AGG_INT_SRC_I2CSMB3);
			break;
		case SMB_CHANNEL_4:
			interrupt_device_ecia_enable_clear(ECIA_AGG_INT_SRC_I2CSMB4);
			break;
		default:
			break;
	}
}/* disable_smb_irq */


#ifndef DISABLE_SMB_MASTER
/******************************************************************************/
/** get_master_state function
 * This function returns the master state
 * @param channel smb channel
 * @return None
 * @note This function is used by any layer which wants to get the master state
 * but doesn't have acess to master object
 ******************************************************************************/
UINT8 get_master_state(const UINT8 channel)
{
    return master[channel].get_state();
}/* get_master_state */
#endif //#ifndef DISABLE_SMB_MASTER

/******************************************************************************/
/** handle_smb_disable_event function
 * This function handles the smb disable event
 * @return TRUE if smb is disabled, FALSE if not
 * @param channel smb channel
 ******************************************************************************/
bool handle_smb_disable(const UINT8 channel)
{
    bool retVal = false;    

#ifndef DISABLE_SMB_MASTER
    if (MASTER_IDLE == master[channel].get_state())
#endif
    {
        if ( base[channel].is_need_to_disable())
        {
            //trace1(0, SMB_CORE, 0, "handle_smb_disable: channel %d",channel);
            smb_disable(channel);
            retVal = true;
        }
    }
    return retVal;
}/* handle_smb_disable */

/******************************************************************************/
/** smb_update_master_wdt_value function
 * This function updates the wdt value in bus clock register
 * @return None
 * @param channel smb channel
 * @param wdt_interval interval in steps of 200ms
 ******************************************************************************/
#ifndef DISABLE_SMB_MASTER
void smb_update_master_wdt_interval(const UINT8 channel,const UINT8 wdt_interval)
{
    master[channel].update_wdt_interval(wdt_interval);
}
#endif

#ifndef DISABLE_ALL_SLAVE_BUFFERS
/******************************************************************************/
/** smb_update_nack_mechanism function
 * This function updates the nack mechanism to use when out of buffers
 * - NACK or CLK_STRETCH
 * @return None
 * @param channel smb channel
 * @param nackMechansim NACK mechanism- NACK or CLK_STRETCH
 ******************************************************************************/
void smb_update_nack_mechanism(const UINT8 channel, const UINT8 nackMechanism)
{
    slave[channel].update_nack_mechanism(nackMechanism);
}
#endif

/******************************************************************************/
/** smb_update_ber_counter_value function
 * This function updates the bus error counter value
 * @param channel smb channel
 * @param value
 * @return None
 ******************************************************************************/
void smb_update_ber_counter_value(const UINT8 channel,const UINT8 value)
{
    base[channel].update_ber_counter_value(value);
}

/******************************************************************************/
/** smb_get_ber_counter_value function
 * This function updates the bus error counter value
 * @param channel smb channel
 * @param value
 * @return None
 ******************************************************************************/
UINT8 smb_get_ber_counter_value(const UINT8 channel)
{
    return base[channel].get_ber_counter_value();
}

/******************************************************************************/
/** smb_update_clk_value function
 * This function updates the clock value in bus clock register
 * @return None
 * @param channel smb channel
 * @param clk_value clock value to be programmed
 ******************************************************************************/
void smb_update_clk_value(const UINT8 channel,const UINT16 clk_value)
{
    base[channel].set_clk_value(clk_value);
}

/******************************************************************************/
/** smb_update_timeoutScaling_value function
 * This function updates the timeout scaling register value
 * @return None
 * @param channel smb channel
 * @param value Timeout Scaling value to be programmed
 ******************************************************************************/
void smb_update_timeoutScaling_value(const UINT8 channel,const UINT32 value)
{
    base[channel].set_timeoutScaling_value(value);
}

/******************************************************************************/
/** smb_enable_timeouts function
 * This function enables timeouts
 * @return None
 * @param channel smb channel
 * @param timeoutsFlag timeout flag as per BYTE0 of completion register
 ******************************************************************************/
extern "C" void smb_enable_timeouts(const UINT8 channel, const UINT8 timeoutsFlag)
{
    base[channel].enable_timeouts(timeoutsFlag);
}

#ifdef HANDLE_POWER_LOW_TO_HIGH_EVENT
/******************************************************************************/
/** handle_power_low_to_high_event function
 * This function handles the
 * @return TRUE if smb is disabled, FALSE if not
 * @param channel smb channel
 ******************************************************************************/
void handle_power_low_to_high_event(const UINT8 channel)
{   
#ifndef DISABLE_SMB_MASTER
    if (MASTER_IDLE == master[channel].get_state())
    {
       if ( base[channel].handle_power_low_to_high_event())
       {             
             //trace1(0, SMB_CORE, 0, "handle_power_low_to_high_event: Channel %d", channel);
           /* inform application if it is retrying current transaction
            * or trying to initiate new transaction
            */
           master[channel].set_completion_status(ERROR_CLK_DATA_NOT_HIGH);
           (void)master[channel].inform_app();
             
             /* schedule timer task for executing port state machine */
#if SMB_FREERTOS           
		   	 smb_raise_timer_event(SMBUS_TIMER_TICK_MSEC);
#elif SMB_SKERN
             kSET_EVENT_WAKETIMER(ms_to_ticks(SMBUS_TIMER_TICK_MSEC), smbus);
             smbTimerTick = ms_to_ticks(SMBUS_TIMER_TICK_MSEC);
#else
#endif        
       }
    }
#else
        if (base[channel].handle_power_low_to_high_event())
        {
            /* schedule timer task for executing port state machine */
#if SMB_FREERTOS
			smb_raise_timer_event(SMBUS_TIMER_TICK_MSEC);

#elif SMB_SKERN
            kSET_EVENT_WAKETIMER(ms_to_ticks(SMBUS_TIMER_TICK_MSEC), smbus);
            smbTimerTick = ms_to_ticks(SMBUS_TIMER_TICK_MSEC);
#else 
#endif
        }
#endif
}/* handle_smb_disable */
#endif

/******************************************************************************/
/**     smb_config_change_event function
* This function handles the configuration change event
* @param channel        smb channel
* @param configValue    configuration value
* @param cfgState       configuration state
* @return None
*******************************************************************************/
void smb_config_change_event(const UINT8 channel,const UINT8 configValue,const UINT8 cfgState)
{

    base[channel].update_config_value(configValue);

    switch(cfgState)
    {
    case CFG_ENABLE_0_TO_1:
        //trace0(0, SMB_CORE, 0, "smb_config_change_event: Enable bit changed from 0 to 1");
        /* Make sure we are not in the middle of disabling smbus controller */
        if (!base[channel].get_disable_flag())
        {
            smb_enable(channel);
        }
        break;

    case CFG_ENABLE_1_TO_0:
        //trace0(0, SMB_CORE, 0, "smb_config_change_event: Enable bit changed from 1 to 0");
#ifndef DISABLE_SMB_MASTER
        if ( master[channel].is_master_idle())
        {
            smb_disable(channel);
        }
        else
                {
                    /* Master is not idle, so wait for current master transaction
                     * to complete before disabling */
                        if (pSmbCallback)
                        {
                                pSmbCallback(channel, SMB_CBK_BUSY, IGNORE_EVENT_VALUE);
                        }
                        base[channel].set_disable_flag(true);
                        //trace0(0, SMB_CORE, 0, "smb_config_change_event: set_disable_flag = TRUE");
                }
#else
        smb_disable(channel);
#endif
        break;
    case CFG_ENABLE_NO_CHANGE:
        //trace0(0, SMB_CORE, 0, "smb_config_change_event: No change in Enable bit");
        break;
    default:
        //trace0(0, SMB_CORE, 0, "smb_config_change_event: Invalid condition");
        break;      
    }
}/* End smb_config_change_event() */

/******************************************************************************/
/**     smb_speed_config_change_event function
* This function handles the smbus speed configuration change event
* @param channel        smb channel
* @param configValue    speed configuration value
* @return None
*******************************************************************************/
void smb_speed_config_change_event(UINT8 channel, UINT8 configValue)
{
    base[channel].update_speed_config_value(configValue);
}

/******************************************************************************/
/**     set_speed function
* This function changes the speed for the controller
* @param channel        smb channel
* @param speed      speed value
* @return None
*******************************************************************************/
void set_speed(UINT8 channel, UINT8 speed)
{
    base[channel].set_speed(speed);
}


/******************************************************************************/
/**     smb_config_change_event function
* This function handles the port configuration change event
* @param channel        smb channel
* @param configValue    smb configuration value
* @return None
*******************************************************************************/
void smb_port_config_change_event(const UINT8 channel,const UINT8 configValue)
{
    base[channel].update_port_config_value(configValue);
}

/******************************************************************************/
/** smb_own_address_changed_event function
 * This function updates the own smb address
 * @param channel smb channel
 * @param value own address value
 * @return None
 ******************************************************************************/
void smb_own_address_changed_event(const UINT8 channel, const UINT16 value)
{
    base[channel].update_own_address(value);
}/* smb_own_address_changed */

/******************************************************************************/
/** smb_port_config_change function
 * This function updates port configuration 
 * @param channel smb channel
 * @param portConfigValue port configuration value
 * @return None
 ******************************************************************************/
void smb_port_config_change(const UINT8 channel,const UINT8 portConfigValue)
{

    base[channel].update_port_config_value(portConfigValue);

}/* smb_port_config_change */

/******************************************************************************/
/** smb_set_default_port function
 * This function sets the default port for the controller 
 * @param channel smb channel
 * @param port the default port
 * @return None
 ******************************************************************************/
void smb_set_default_port(const UINT8 channel,const UINT8 port)
{

    base[channel].set_default_port(port);

}/* smb_port_config_change */

#ifdef HANDLE_POWER_LOW_TO_HIGH_EVENT
/******************************************************************************/
/** smb_power_low_to_high_event_set function
 * This function is invoked if power changes from low to high
 * @param None
  * @return None
 ******************************************************************************/
void smb_power_low_to_high_event_set(void)
{
    UINT8 channel;

    /* For each channel */
    for (channel=0;channel<MAX_SMB;channel++)
    {
          base[channel].set_power_low_to_high_event(true);
          handle_power_low_to_high_event(channel);
    }
}
#endif

/** @}
 */

