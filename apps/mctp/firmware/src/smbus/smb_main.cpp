/*****************************************************************************
* � 2018 Microchip Technology Inc. and its subsidiaries.
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
/** @defgroup smb smb_main
 *  @{
 */
/** @file smb_main.cpp
 \brief the smb main cpp file
 This file is the main interface to smbus driver

<b>Platform:</b> This is ARM-based component
  Details of hardware blocks on which this smbus reuse component is tested:
    smbus HW Block name - blk_i2c_spb, smbus HW Block build # - B0076   
    dma HW Block name - blk_dma_ahb, dma HW Block build # - B0007
    FPGA build used for testing -> MEC1322 FPGA build Rev-81-Randy - 081_chip_top_fpga2_run1
    //depotAE/projects/MEC1322/FPGA builds/Rev-81-Randy/081_chip_top_fpga2_run1.mcs

<b>Toolset:</b> MDK-Lite V 4.23; Armcc, Armasm, ArmLink, ArmAr, FromElf - V4.1.0.894 
<b>Reference:</b> smsc_reusable_fw_requirement.doc */

/*******************************************************************************
 $File: //depot_pcs/FWEng/solutions/cec1712_GEN2/maincodeline/secureboot_app/src/plib/smbus/smb_main.cpp $

Version Control Information (Perforce)
*******************************************************************************
$Revision: #2 $
$DateTime: 2020/07/24 01:41:32 $
$Author: I19961 $
  Change Description:
    2. Code changes to support Master Tx chaining
    1. Branched from //depot_pcs/FWEng/projects/MEC1324/maincodeline/libSmbus/source/smb/ 
        label - MEC1324_LIB_SMBUS_0400
*******************************************************************************/
extern "C"
{
    #include <common.h>
    #include "interrupt/interrupt_api.h"
    #include "rtos_definitions.h"
}

#include "smb_config.h"
#include "smb.h"
#include "smb_core.h"

#ifdef HANDLE_POWER_LOW_TO_HIGH_EVENT
bool prevPowerOn= false;
#endif

/* force all global data into .data */
UINT32 smbTimerTick;

SMB_CALLBACK_FUNC_PTR pSmbCallback;


#if SMB_FREERTOS
EventGroupHandle_t *pSmb_event_flag_handle; //Event flags used by smbus
#endif



#if SMB_FREERTOS
/* Add here smbus freertos related*/
extern "C" void smbus_main(void *pvParameters) ;
#elif SMB_SKERN
extern "C" void smbus_main_task(enum EVENT_TYPE call_type);
#else 

#endif
extern "C" UINT8 smb_deregister_slave(const UINT8 channel, SLAVE_FUNC_PTR slaveFuncPtr);
extern "C" void smbus_init_task(void);
extern "C" void smb_check_master_wdt(void);

#if SMB_FREERTOS
/******************************************************************************/

/** smb_get_current_timestamp.
 * Hook function to retrieve the current timestamp; customize as per RTOS port
 * @param None
 * @return 16-bit timestamp value
 ******************************************************************************/
extern "C" UINT16 smb_get_current_timestamp(void) {
    trace0(0, SMBUS, 0, "smb_get_current_timestamp: Enter");

    return (UINT16) (xTaskGetTickCount() / 10);
    //return (UINT16)(tx_time_get());  //ThreadX version
    //return (UINT16)(kGET_TICKS());   //SKERN version
}

/******************************************************************************/

/** smb_raise_timer_event.
 * Hook function to trigger timer events; customize as per RTOS port
 * @param timeInMs time units in milliseconds
 * @return None
 ******************************************************************************/
extern "C" void smb_raise_timer_event(const UINT8 timeInMs) {
    //trace0(0, SMBUS, 0, "smb_raise_timer_event: Enter");

    //kSET_EVENT_WAKETIMER(ms_to_ticks(SMBUS_TIMER_TICK_MSEC), smbus);
    smbTimerTick = ms_to_ticks(SMBUS_TIMER_TICK_MSEC);
}

/******************************************************************************/

/** smb_raise_interrupt_event.
 * Hook function to trigger interrupt events; customize as per RTOS port
 * @param None
 * @return None
 *******************************************************************************/
extern "C" void smb_raise_interrupt_event(void) {

    BaseType_t xHigherPriorityTaskWoken_smbus, xResult;

    //trace0(0, SMBUS, 0, "smb_raise_interrupt_event: Enter");

    //Note: This function will be called only from ISR context - smb_core_isr or smb_dma_isr

    //kSET_EVENT_INTERRUPT(smbus);  //SKERN version
    //tx_event_flags_set(pSmb_event_flag, SMBUS_EVENT_ISR, TX_OR); //ThreadX version

    /* xHigherPriorityTaskWoken must be initialised to pdFALSE. */
    xHigherPriorityTaskWoken_smbus = pdFALSE;

    //Abhishek 
    /* Set bit 0 and bit 4 in xEventGroup. */
    xResult = xEventGroupSetBitsFromISR(*pSmb_event_flag_handle, /* The event group being updated. */
            SMBUS_EVENT_ISR, /* The bits being set. */
            &xHigherPriorityTaskWoken_smbus);

    if (xResult != pdFAIL) {
        portEND_SWITCHING_ISR(xHigherPriorityTaskWoken_smbus);
    }

}

#endif

/******************************************************************************/
/** smbus_configure_and_enable function.
 * This function can be used to start and enable the smbus controller
 * @param channel - channel, check enums in smb.h
 * @param own_address - 7-bit smb address
 * @param speed - speed, check enums in smb.h
 * @param port - default port on the controller
 * @return None
*******************************************************************************/
extern "C" void smbus_configure_and_enable(UINT8 channel, UINT16 own_address, UINT8 speed, UINT8 port, UINT8 configFlag)
{
    //trace12(0, SMBUS, 0, "SMBUS_CONFIGURE_AND_ENABLE: channel %d port %d", channel, port );
#ifndef DISABLE_SMB_MASTER
    smb_update_master_wdt_interval(channel, 0xA);
#endif
    smb_own_address_changed_event(channel, own_address);
    smb_speed_config_change_event(channel, speed);
    smb_set_default_port(channel, port);
    smb_config_change_event(channel, configFlag, CFG_ENABLE_0_TO_1);
//  smb_enable_timeouts(channel,0x4);
}

/******************************************************************************/
/** smbus_disable function.
 * This function can be used to disable the smbus controller.
 * Make the sure the controller was enabled prior to using this function
 * @param channel - channel, check enums in smb.h 
 * @return None
*******************************************************************************/
extern "C" void smbus_disable(UINT8 channel)
{
    smb_config_change_event(channel, 0x0, CFG_ENABLE_1_TO_0);   
}

#if SMB_FREERTOS
/******************************************************************************/

/**   smbus_app_timer - smbus application timer function
 * @param None
 * @return None
 *******************************************************************************/
extern "C" void smbus_app_timer(TimerHandle_t pxTimer) {
    //trace0(0, SMBUS, 0, "smbus_app_timer: Enter");

    if (0 != smbTimerTick) {
        smbTimerTick--;
        if (0 == smbTimerTick) {
            //trace0(0, SMBUS, 0, "smbus_app_timer: set SMBUS_EVENT_TIMER");
            xEventGroupSetBits(*pSmb_event_flag_handle, SMBUS_EVENT_TIMER);
        }
    }

#ifndef DISABLE_SMB_MASTER
    xEventGroupSetBits(*pSmb_event_flag_handle, SMBUS_EVENT_MONITOR_WDT);
#endif
}
#endif

#if 0
/******************************************************************************/
/**   smbus_app_timer - smbus application timer function
* @param None
* @return None
*******************************************************************************/
extern "C" void smbus_app_timer(void)
{       
      //trace0(0, SMBUS, 0, "smbus_app_timer: Enter");
    
        if (0 != smbTimerTick)
        {
            smbTimerTick--;
            if (0 == smbTimerTick)
            {   
                //trace0(0, SMBUS, 0, "smbus_app_timer: set SMBUS_EVENT_TIMER");
                tx_event_flags_set(pSmb_event_flag, SMBUS_EVENT_TIMER, TX_OR);          
            }       
        }

    #ifndef DISABLE_SMB_MASTER
        tx_event_flags_set(pSmb_event_flag, SMBUS_EVENT_MONITOR_WDT, TX_OR);
    #endif
}
#endif

/******************************************************************************/
/**     smbus_init_task
* @param None
* @return None
*******************************************************************************/
extern "C" void smbus_init_task(void)
{
    //trace0(0, SMB_T, 0, "smbus_init_task: Enter");
    smbTimerTick = 0;
    pSmbCallback = NULL;
    smbus_core_init_task();   
    
    
}

#if SMB_FREERTOS

/******************************************************************************/

/**     smb_register_eventFlag_and_callback
 * @param ptr_event_flag pointer to RTOS event flag structure
 * @param pCallback pointer to callback function 
 * @return None
 *******************************************************************************/
extern "C" void smb_register_eventFlag_and_callback(EventGroupHandle_t *ptr_event_flag_handle, SMB_CALLBACK_FUNC_PTR pCallback) {
    pSmbCallback = pCallback;
    pSmb_event_flag_handle = ptr_event_flag_handle;
}


/******************************************************************************/

/**   smbus_main - SMBus main task
 * @param None
 * @return None
 *******************************************************************************/
extern "C" void smbus_main(void *pvParameters) {
    EventBits_t uxBits;


    //trace0(0, SMBUS, 0, "smbus_main_task: Enter smbus_main() task - RTOS");

    uint32_t watermark = uxTaskGetStackHighWaterMark(NULL);

    while (1) {


        /* Wait for skern_event_flags.  */
        //trace0(0, SMBUS, 0, "smbus_main: wait on events ");
        /* This event should be set on any event requiring a smbus task schedule */
        //uxBits = xEventGroupWaitBits(*pSmb_event_flag_handle, 0xF, pdTRUE, pdFALSE, portMAX_DELAY);
        //        uxBits = xEventGroupWaitBits(*pSmb_event_flag_handle, 0xF, pdTRUE, pdFALSE, pdMS_TO_TICKS(5));
        uxBits = xEventGroupWaitBits(*pSmb_event_flag_handle, 0xF, pdTRUE, pdFALSE, portMAX_DELAY);

        //TRACE2(141, SMBUS, 0, "smbus_main: uxBits = %04Xh %04Xh ", uxBits>>16, uxBits);

        if (uxBits & SMBUS_EVENT_ISR) {
            //trace0(0, SMBUS, 0, "handle SMBUS_EVENT_ISR ");
            smbus_interrupt_task();
        }

        if (uxBits & SMBUS_EVENT_TIMER) {
            //trace0(0, SMBUS, 0, "handle SMBUS_EVENT_TIMER ");
            
            smbus_timer_task();
        }

#ifndef DISABLE_SMB_MASTER
        if (uxBits & SMBUS_EVENT_MONITOR_WDT) {
            //TRACE0(144, SMBUS, 0, "handle SMBUS_EVENT_MONITOR_WDT ");
            smb_check_master_wdt();
        }
#endif

        watermark = uxTaskGetStackHighWaterMark(NULL);
//        AppUARTDebugUpdateItem((size_t)&(((UART_DEBUG_t *) 0)->os.minStackFree_SMBUS), &watermark, 4);

    }
    
}/* End smbus_main_task() */

#elif SMB_SKERN
/******************************************************************************/
/**     SMBus main task
* @param call_type      Event Type - Interrupt/Task/Timer
* @return None
*******************************************************************************/
extern "C" VOID smbus_main_task(enum EVENT_TYPE call_type)
{
    //trace0(0, SMB_T, 0, "smbus_main_task: Enter");
    switch (call_type)
    {
    case EventTypeInit:
        smbus_init_task();
        break;
    case EventTypeInterrupt:
        //trace0(0, SMB_MAIN, 0, "smbus_main_task: EventTypeInterrupt");
        smbus_interrupt_task();
        break;
    case EventTypeTask:
        //trace0(0, SMB_MAIN, 0, "smbus_main_task: EventTypeTask");
    #ifndef DISABLE_ALL_SLAVE_BUFFERS
        #ifndef DISABLE_PENDING_PACKETS
            smbus_event_task();
        #endif
    #endif
        break;
    case EventTypeTimer:
        //trace0(0, SMB_T, 0, "smbus_main_task: EventTypeTimer");
        smbus_timer_task();
        break;
    default:
        break;
    }
    return;
}/* End smbus_main_task() */
#else
#endif /* SMBUS FREERTOS Modifications */

#ifndef DISABLE_SMB_MASTER
/******************************************************************************/
/** smb_busyStatus_get function.
 * This function checks if master resource is available on the default port
 * @param channel smb channel
 * @return MASTER_AVAILABLE/MASTER_BUSY
*******************************************************************************/
extern "C" UINT8 smb_busyStatus_get(const UINT8 channel)
{
    return check_master_availablity(channel, true, 0);
}

/******************************************************************************/
/** smb_portBusyStatus_get function.
 * This function checks if master resource is available on the queried port
 * @param channel smb channel
 * @param port the port to check if it is available
 * @return MASTER_AVAILABLE/MASTER_BUSY
*******************************************************************************/
extern "C" UINT8 smb_portBusyStatus_get(const UINT8 channel,const UINT8 port)
{
    return check_master_availablity(channel, false, port);
}

/******************************************************************************/
/** smbus_change_port function
 * This function changes the controller port
 * @param channel smb channel
 * @param channel the port to configure
 * @return MASTER_ERROR/MASTER_OK
 ******************************************************************************/
extern "C" UINT8 smb_change_port(const UINT8 channel,const UINT8 port)
{
    return change_port(channel, port);
}

/******************************************************************************/
/** smb_set_speed function
 * This function changes smbus speed
 * @param channel smb channel
 * @param speed the speed value
 * @return MASTER_ERROR/MASTER_OK
 ******************************************************************************/
extern "C" void smb_set_speed(const UINT8 channel,const UINT8 speed)
{
     set_speed(channel, speed);
}

#if 0
/******************************************************************************/
/**     Initiates smbus master operation which is blocking i.e. returns only 
*       when the transaction is complete
* @param channel        channel
* @param buffer_ptr     Buffer for the smbus transaction
* @param smb_protocol   smbus protocol byte
* @param pecEnable      Flag to enable/disable PEC
* @param WriteCount     Number of bytes to transmit
* @param func_ptr       Function to call after success/failure of the transaction
*                       This function will be invoked on completion of transaction
*                       with status. The function type is:
*                       typedef void (*MASTER_FUNC_PTR)(UINT8, UINT8 *)
*                       The first parameter is the status, the next is the
*                       buffer_ptr that was passed
* @return               MASTER_OK on success, MASTER_ERROR if smbus is not ready for
*                       master mode operation
* @note     This function is called by the application whenever it wants to
*           initiate a master transaction on the smbus. If this function returns
*           MASTER_ERROR, application could retry after some time
*           For ReadBlock protocol the application should provide a 80 byte
*           buffer.
*******************************************************************************/
extern "C" UINT8 smb_protocol_execute_blocking(const UINT8 channel, UINT8 *buffer_ptr,const UINT8 smb_protocol
                                        ,const UINT8 writeCount,const UINT8 pecEnable, MASTER_FUNC_PTR func_ptr, const UINT8 readChainedFlag, const UINT8 writeChainedFlag)
{
    UINT8 retVal = (UINT8)MASTER_ERROR; 

    if ( execute_master_request(channel, buffer_ptr, smb_protocol, writeCount, (bool)pecEnable, func_ptr, true, readChainedFlag, writeChainedFlag))
    {
        retVal = (UINT8)MASTER_OK;
    }   

    return retVal;

}/* End smb_protocol_execute() */
#endif

/******************************************************************************/
/**     Initiates smbus master operation
* @param channel        channel
* @param buffer_ptr     Buffer for the smbus transaction
* @param smb_protocol   smbus protocol byte
* @param pecEnable      Flag to enable/disable PEC
* @param WriteCount     Number of bytes to transmit
* @param func_ptr       Function to call after success/failure of the transaction
*                       This function will be invoked on completion of transaction
*                       with status. The function type is:
*                       typedef void (*MASTER_FUNC_PTR)(UINT8, UINT8 *)
*                       The first parameter is the status, the next is the
*                       buffer_ptr that was passed
* @param readChainedFlag     flag to indicate if read needs to be done using dma chaining
* @param writeChainedFlag    flag to indicate if write needs to be done using dma chaining
* @return               MASTER_OK on success, MASTER_ERROR if smbus is not ready for
*                       master mode operation
* @note     This function is called by the application whenever it wants to
*           initiate a master transaction on the smbus. If this function returns
*           MASTER_ERROR, application could retry after some time
*           For ReadBlock protocol the application should provide a 80 byte
*           buffer.
*******************************************************************************/
extern UINT8 smb_protocol_execute(const UINT8 channel, UINT8 *buffer_ptr,const UINT8 smb_protocol, const UINT8 writeCount,
            const UINT8 pecEnable, MASTER_FUNC_PTR func_ptr, const UINT8 readChainedFlag, const UINT8 writeChainedFlag)
{
    UINT8 retVal = (UINT8)MASTER_ERROR;

    if ( execute_master_request(channel, buffer_ptr, smb_protocol, writeCount, 
             (bool)pecEnable, func_ptr, false, (bool)readChainedFlag, (bool)writeChainedFlag ))
    {
        retVal = (UINT8)MASTER_OK;
    }       

    return retVal;

}/* End smb_protocol_execute() */
#endif

#ifndef DISABLE_ALL_SLAVE_BUFFERS
/******************************************************************************/
/** smb_register_slave function.
 * This function registers a smbus slave application
 * @channel the channel on which the slave is registered
 * @param slaveFuncPtr   The application function to call on receiving a packet
 * @return  STATUS_OK on successful registration, else error status
 * @note    Whenever a application expects data from smbus (i.e acts as slave) it
 *          needs to register using this function.
 *          The application function that is registered should only copy the
 *          packet from smbus buffer, it should not the process the data in that
 *          function
*******************************************************************************/
extern "C" UINT8 smb_register_slave(const UINT8 channel, SLAVE_FUNC_PTR slaveFuncPtr)
{
    return smb_core_register_slave(channel, slaveFuncPtr);
}/* smb_register_slave */

/******************************************************************************/
/** smb_deregister_slave function.
 * This function is used de-register a smbus slave application
 * @channel the channel on which the slave was registered
 * @param   slaveFuncPtr   Application function pointer
 * @return  STATUS_OK on successful de-registration, else error status
*******************************************************************************/
extern "C" UINT8 smb_deregister_slave(const UINT8 channel, SLAVE_FUNC_PTR slaveFuncPtr)
{
    return smb_core_deregister_slave(channel, slaveFuncPtr);
}/* smb_register_slave */
#endif

#ifndef DISABLE_SMB_MASTER
/******************************************************************************/
/** smb_check_master_wdt
 * This function handles master wdt processing. It needs to be called periodically
 * @return None
 * @param None
*******************************************************************************/
extern "C" void smb_check_master_wdt(void)
{
    UINT8 channel;

    /* For each channel */
    for (channel=0;channel<MAX_SMB;channel++)
    {
        smb_core_master_wdt(channel);
    }
}/* End smb_check_master_wdt() */
#endif

#ifdef HANDLE_POWER_LOW_TO_HIGH_EVENT
/******************************************************************************/
/** smb_power_on
 * @param  isON   1 power is on, 0 power is off
* @return none
* @note  This API will be called whenever power state is changed
*******************************************************************************/
extern "C" void smb_power_on(const UINT8 isON)
{

    if ((!prevPowerOn) && (isON))
    {
        //trace0(0, SMBUS, 0, "smb_power_on ");

        smb_power_low_to_high_event_set();
    }

    prevPowerOn = (bool)isON;
}/* End smb_power_on() */
#endif

/******************************************************************************/
/** smb_isr
 * The main entry point for smbus interrupt service routine
 * @return None
 * @param None
*******************************************************************************/
extern "C" void smb_isr(void)
{
    register uint8_t channel = 0;
    for ( ; channel < MAX_SMB; ++channel)
    {
		switch(channel)
		{
			case SMB_CHANNEL_1:
				if (interrupt_device_ecia_result_get(ECIA_AGG_INT_SRC_I2CSMB1))
				{
					/* check if DMA interrupt occurs simultaneously */
					/* process DMA ISR firstly to fix boundary corner case */
					/* slave dma's start from Even channel numbers 1, 3, 5, 7, 9 */
					if (interrupt_device_ecia_result_get(ECIA_AGG_INT_SRC_DMA_CH03))
					{
						smb_core_isr_dma((UINT8)channel, false);     //true for slave
						// Clear smbus (channel) IRQ status.
						interrupt_device_ecia_source_clear(ECIA_AGG_INT_SRC_DMA_CH03);
					}

					/* master dma's start from odd channel numbers 0, 2, 4, 6, 8 */
					if (interrupt_device_ecia_result_get(ECIA_AGG_INT_SRC_DMA_CH02))
					{
						smb_core_isr_dma((UINT8)channel, false);
						// Clear smbus (channel) IRQ status.
						interrupt_device_ecia_source_clear(ECIA_AGG_INT_SRC_DMA_CH02);
					}
					smb_core_isr(channel);
					// Clear smbus (channel) IRQ status.
					interrupt_device_ecia_source_clear(ECIA_AGG_INT_SRC_I2CSMB1);
				}
			break;
			case SMB_CHANNEL_2:
				if (interrupt_device_ecia_result_get(ECIA_AGG_INT_SRC_I2CSMB2))
				{
					/* check if DMA interrupt occurs simultaneously */
					/* process DMA ISR firstly to fix boundary corner case */
					/* slave dma's start from Even channel numbers 1, 3, 5, 7, 9 */
					if (interrupt_device_ecia_result_get(ECIA_AGG_INT_SRC_DMA_CH05))
					{
						smb_core_isr_dma((UINT8)channel, false);     //true for slave
						// Clear smbus (channel) IRQ status.
						interrupt_device_ecia_source_clear(ECIA_AGG_INT_SRC_DMA_CH05);
					}

					/* master dma's start from odd channel numbers 0, 2, 4, 6, 8 */
					if (interrupt_device_ecia_result_get(ECIA_AGG_INT_SRC_DMA_CH04))
					{
						smb_core_isr_dma((UINT8)channel, false);
						// Clear smbus (channel) IRQ status.
						interrupt_device_ecia_source_clear(ECIA_AGG_INT_SRC_DMA_CH04);
					}
					smb_core_isr(channel);
					// Clear smbus (channel) IRQ status.
					interrupt_device_ecia_source_clear(ECIA_AGG_INT_SRC_I2CSMB2);
				}
			break;
			case SMB_CHANNEL_3:
				if (interrupt_device_ecia_result_get(ECIA_AGG_INT_SRC_I2CSMB3))
				{
					/* check if DMA interrupt occurs simultaneously */
					/* process DMA ISR firstly to fix boundary corner case */
					/* slave dma's start from Even channel numbers 1, 3, 5, 7, 9 */
					if (interrupt_device_ecia_result_get(ECIA_AGG_INT_SRC_DMA_CH07))
					{
						smb_core_isr_dma((UINT8)channel, false);     //true for slave
						// Clear smbus (channel) IRQ status.
						interrupt_device_ecia_source_clear(ECIA_AGG_INT_SRC_DMA_CH07);
					}

					/* master dma's start from odd channel numbers 0, 2, 4, 6, 8 */
					if (interrupt_device_ecia_result_get(ECIA_AGG_INT_SRC_DMA_CH06))
					{
						smb_core_isr_dma((UINT8)channel, false);
						// Clear smbus (channel) IRQ status.
						interrupt_device_ecia_source_clear(ECIA_AGG_INT_SRC_DMA_CH06);
					}
					smb_core_isr(channel);
					// Clear smbus (channel) IRQ status.
					interrupt_device_ecia_source_clear(ECIA_AGG_INT_SRC_I2CSMB3);
				}
			break;
			case SMB_CHANNEL_4:
				if (interrupt_device_ecia_result_get(ECIA_AGG_INT_SRC_I2CSMB4))
				{
					/* check if DMA interrupt occurs simultaneously */
					/* process DMA ISR firstly to fix boundary corner case */
					/* slave dma's start from Even channel numbers 1, 3, 5, 7, 9 */
					if (interrupt_device_ecia_result_get(ECIA_AGG_INT_SRC_DMA_CH09))
					{
						smb_core_isr_dma((UINT8)channel, false);     //true for slave
						// Clear smbus (channel) IRQ status.
						interrupt_device_ecia_source_clear(ECIA_AGG_INT_SRC_DMA_CH09);
					}

					/* master dma's start from odd channel numbers 0, 2, 4, 6, 8 */
					if (interrupt_device_ecia_result_get(ECIA_AGG_INT_SRC_DMA_CH08))
					{
						smb_core_isr_dma((UINT8)channel, false);
						// Clear smbus (channel) IRQ status.
						interrupt_device_ecia_source_clear(ECIA_AGG_INT_SRC_DMA_CH08);
					}
					smb_core_isr(channel);
					// Clear smbus (channel) IRQ status.
					interrupt_device_ecia_source_clear(ECIA_AGG_INT_SRC_I2CSMB4);
				}
			break;
			case SMB_CHANNEL_0:
			/* No break */
			default :
				if (interrupt_device_ecia_result_get(ECIA_AGG_INT_SRC_I2CSMB0))
				{
					/* check if DMA interrupt occurs simultaneously */
					/* process DMA ISR firstly to fix boundary corner case */
					/* slave dma's start from Even channel numbers 1, 3, 5, 7, 9 */
					if (interrupt_device_ecia_result_get(ECIA_AGG_INT_SRC_DMA_CH01))
					{
						smb_core_isr_dma((UINT8)channel, false);     //true for slave
						// Clear smbus (channel) IRQ status.
						interrupt_device_ecia_source_clear(ECIA_AGG_INT_SRC_DMA_CH01);
					}

					/* master dma's start from odd channel numbers 0, 2, 4, 6, 8 */
					if (interrupt_device_ecia_result_get(ECIA_AGG_INT_SRC_DMA_CH00))
					{
						smb_core_isr_dma((UINT8)channel, false);
						// Clear smbus (channel) IRQ status.
						interrupt_device_ecia_source_clear(ECIA_AGG_INT_SRC_DMA_CH00);
					}
					smb_core_isr(channel);
					// Clear smbus (channel) IRQ status.
					interrupt_device_ecia_source_clear(ECIA_AGG_INT_SRC_I2CSMB0);
				}
			break;
		}
    }
}/* smb_isr */

/******************************************************************************/
/** smb_dma_isr
 * This is currently used only for the slave
 * @return None
 * @param None
*******************************************************************************/
extern "C" void smb_dma_isr(void)
{
    register uint8_t channel = 0;
    
    //TRACE0(48, SMB_MAIN, 0, "smb_dma_isr: enter");
    
    for ( ; channel < MAX_SMB; ++channel)
    {
		switch(channel)
		{
			case SMB_CHANNEL_1:
				/* slave dma's start from Even channel numbers 1, 3, 5, 7, 9 */
				if (interrupt_device_ecia_result_get(ECIA_AGG_INT_SRC_DMA_CH03))
				{
					smb_core_isr_dma((UINT8)channel, true);     //true for slave
					// Clear smbus (channel) IRQ status.
					interrupt_device_ecia_source_clear(ECIA_AGG_INT_SRC_DMA_CH03);
				}

				/* master dma's start from odd channel numbers 0, 2, 4, 6, 8 */
				if (interrupt_device_ecia_result_get(ECIA_AGG_INT_SRC_DMA_CH02))
				{
					smb_core_isr_dma((UINT8)channel, true);
					// Clear smbus (channel) IRQ status.
					interrupt_device_ecia_source_clear(ECIA_AGG_INT_SRC_DMA_CH02);
				}
			break;
			case SMB_CHANNEL_2:
				/* slave dma's start from Even channel numbers 1, 3, 5, 7, 9 */
				if (interrupt_device_ecia_result_get(ECIA_AGG_INT_SRC_DMA_CH05))
				{
					smb_core_isr_dma((UINT8)channel, false);     //true for slave
					// Clear smbus (channel) IRQ status.
					interrupt_device_ecia_source_clear(ECIA_AGG_INT_SRC_DMA_CH05);
				}

				/* master dma's start from odd channel numbers 0, 2, 4, 6, 8 */
				if (interrupt_device_ecia_result_get(ECIA_AGG_INT_SRC_DMA_CH04))
				{
					smb_core_isr_dma((UINT8)channel, false);
					// Clear smbus (channel) IRQ status.
					interrupt_device_ecia_source_clear(ECIA_AGG_INT_SRC_DMA_CH04);
				}
			break;
			case SMB_CHANNEL_3:
				/* slave dma's start from Even channel numbers 1, 3, 5, 7, 9 */
				if (interrupt_device_ecia_result_get(ECIA_AGG_INT_SRC_DMA_CH07))
				{
					smb_core_isr_dma((UINT8)channel, true);     //true for slave
					// Clear smbus (channel) IRQ status.
					interrupt_device_ecia_source_clear(ECIA_AGG_INT_SRC_DMA_CH07);
				}

				/* master dma's start from odd channel numbers 0, 2, 4, 6, 8 */
				if (interrupt_device_ecia_result_get(ECIA_AGG_INT_SRC_DMA_CH06))
				{
					smb_core_isr_dma((UINT8)channel, true);
					// Clear smbus (channel) IRQ status.
					interrupt_device_ecia_source_clear(ECIA_AGG_INT_SRC_DMA_CH06);
				}
			break;
			case SMB_CHANNEL_4:
				/* slave dma's start from Even channel numbers 1, 3, 5, 7, 9 */
				if (interrupt_device_ecia_result_get(ECIA_AGG_INT_SRC_DMA_CH09))
				{
					smb_core_isr_dma((UINT8)channel, false);     //true for slave
					// Clear smbus (channel) IRQ status.
					interrupt_device_ecia_source_clear(ECIA_AGG_INT_SRC_DMA_CH09);
				}

				/* master dma's start from odd channel numbers 0, 2, 4, 6, 8 */
				if (interrupt_device_ecia_result_get(ECIA_AGG_INT_SRC_DMA_CH08))
				{
					smb_core_isr_dma((UINT8)channel, false);
					// Clear smbus (channel) IRQ status.
					interrupt_device_ecia_source_clear(ECIA_AGG_INT_SRC_DMA_CH08);
				}
			break;
			case SMB_CHANNEL_0:
			/* No break */
			default:
				/* slave dma's start from Even channel numbers 1, 3, 5, 7, 9 */
				if (interrupt_device_ecia_result_get(ECIA_AGG_INT_SRC_DMA_CH01))
				{
					smb_core_isr_dma((UINT8)channel, false);     //true for slave
					// Clear smbus (channel) IRQ status.
					interrupt_device_ecia_source_clear(ECIA_AGG_INT_SRC_DMA_CH01);
				}

				/* master dma's start from odd channel numbers 0, 2, 4, 6, 8 */
				if (interrupt_device_ecia_result_get(ECIA_AGG_INT_SRC_DMA_CH00))
				{
					smb_core_isr_dma((UINT8)channel, false);
					// Clear smbus (channel) IRQ status.
					interrupt_device_ecia_source_clear(ECIA_AGG_INT_SRC_DMA_CH00);
				}
			break;
		}
    }
    
}/* smb_dma_isr */


/******************************************************************************/
/** smb_task_wait_event_bits();
* Wait for bit set from MCTP, bit is set once MCTP receives and transmits to application
* layer
* @param event_bits - Event bits to be waited
* @return uint32_t event bits waited or Timeout
*******************************************************************************/
uint32_t smb_task_wait_event_bits(uint32_t event_bits, uint32_t wait_all_bits)
{  
	uint32_t uxBits = 0;

    uxBits = xEventGroupWaitBits(*pSmb_event_flag_handle,
                        event_bits , pdTRUE, wait_all_bits, portMAX_DELAY);
    return uxBits;
}

/** @}
*/
