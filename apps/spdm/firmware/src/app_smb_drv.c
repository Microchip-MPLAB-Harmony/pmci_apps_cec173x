/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app_smb_drv.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It
    implements the logic of the application's state machine and it may call
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "app_smb_drv.h"

#include "definitions.h"
#include "rtos_definitions.h"
#include "smb.h"
#include "mctp.h"
// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_SMB_DRV_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

APP_SMB_DRV_DATA app_smb_drvData;

EventGroupHandle_t xSmbusEventGroupHandle;
SemaphoreHandle_t xSmbSemaphore = NULL;
TimerHandle_t xSmbusTimer;

extern void smbus_app_timer(TimerHandle_t pxTimer);
extern void smbus_main(void *pvParameters);
extern void smb_register_eventFlag_and_callback(EventGroupHandle_t *ptr_event_flag_handle, 
                                    SMB_CALLBACK_FUNC_PTR pCallback);

extern void smb_isr(void);
extern void smb_dma_isr(void);
// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************
static void I2CSMB_GRP_InterruptHandler (void);
static void DMA_GRP_InterruptHandler (void);

void I2CSMB0_GRP_InterruptHandler       ( void ) __attribute__((alias("I2CSMB_GRP_InterruptHandler")));
void I2CSMB1_GRP_InterruptHandler       ( void ) __attribute__((alias("I2CSMB_GRP_InterruptHandler")));
void I2CSMB2_GRP_InterruptHandler       ( void ) __attribute__((alias("I2CSMB_GRP_InterruptHandler")));
void I2CSMB3_GRP_InterruptHandler       ( void ) __attribute__((alias("I2CSMB_GRP_InterruptHandler")));
void I2CSMB4_GRP_InterruptHandler       ( void ) __attribute__((alias("I2CSMB_GRP_InterruptHandler")));
 
void DMA_CH00_GRP_InterruptHandler       ( void ) __attribute__((alias("DMA_GRP_InterruptHandler")));
void DMA_CH01_GRP_InterruptHandler       ( void ) __attribute__((alias("DMA_GRP_InterruptHandler")));
void DMA_CH02_GRP_InterruptHandler       ( void ) __attribute__((alias("DMA_GRP_InterruptHandler")));
void DMA_CH03_GRP_InterruptHandler       ( void ) __attribute__((alias("DMA_GRP_InterruptHandler")));
void DMA_CH04_GRP_InterruptHandler       ( void ) __attribute__((alias("DMA_GRP_InterruptHandler")));
void DMA_CH05_GRP_InterruptHandler       ( void ) __attribute__((alias("DMA_GRP_InterruptHandler")));
void DMA_CH06_GRP_InterruptHandler       ( void ) __attribute__((alias("DMA_GRP_InterruptHandler")));
void DMA_CH07_GRP_InterruptHandler       ( void ) __attribute__((alias("DMA_GRP_InterruptHandler")));
void DMA_CH08_GRP_InterruptHandler       ( void ) __attribute__((alias("DMA_GRP_InterruptHandler")));
void DMA_CH09_GRP_InterruptHandler       ( void ) __attribute__((alias("DMA_GRP_InterruptHandler")));

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_SMB_DRV_Initialize ( void )

  Remarks:
    See prototype in app_smb_drv.h.
 */

void APP_SMB_DRV_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    app_smb_drvData.state = APP_SMB_DRV_STATE_INIT;



    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_SMB_DRV_Tasks ( void )

  Remarks:
    See prototype in app_smb_drv.h.
 */

void APP_SMB_DRV_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( app_smb_drvData.state )
    {
        /* Application's initial state. */
        case APP_SMB_DRV_STATE_INIT:
        {
            bool appInitialized = true;   
            xSmbusEventGroupHandle = xEventGroupCreate();
            
            xSmbusTimer = xTimerCreate("smb_timer",      // Text name for the task.  Helps debugging only.  Not used by FreeRTOS.
                           10,                  // The period of the timer in ticks.
                           pdTRUE,               // This is an auto-reload timer.
                           NULL,               // A variable incremented by the software timer's callback function
                           smbus_app_timer);      // The function to execute when the timer expires.
            
            xSmbSemaphore = xSemaphoreCreateBinary();   
            if ((xSmbusEventGroupHandle == NULL) || 
                  (xSmbusTimer == NULL) || 
                  (xSmbSemaphore == NULL))
            {
                 appInitialized = false;
                 app_smb_drvData.state = APP_SMB_DRV_STATE_INIT_ERROR;
                 break;
            }
            
            if (xTimerStart(xSmbusTimer, 0) != pdPASS) 
            {
                appInitialized = false;
                app_smb_drvData.state = APP_SMB_DRV_STATE_INIT_ERROR;
                break;
            }
            
            if (appInitialized)
            {
            xSemaphoreGive(xSmbSemaphore);   
            /* This function initializes the timer tick and calls the SMBus Core Init 
             Task as a part of SDK Functions*/
            smbus_init_task();   
            /* Register the Event Flag Group and also Register the Callback function */
            smb_register_eventFlag_and_callback(&xSmbusEventGroupHandle, 
                           (SMB_CALLBACK_FUNC_PTR) smb_callback);   
                app_smb_drvData.state = APP_SMB_DRV_STATE_SERVICE_TASKS;
            }
            break;
        } 
 
        case APP_SMB_DRV_STATE_SERVICE_TASKS:
        {
            smbus_main((void*)NULL);
            break;
        }

        case APP_SMB_DRV_STATE_INIT_ERROR:
        /*  no break */
        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

uint8_t mctp_i2c_tx(const uint8_t channel, 
                    uint8_t *buffer_ptr, 
                    const uint8_t smb_protocol, 
                    const uint8_t writeCount, 
                    const uint8_t pecEnable, 
                    I2C_MASTER_FUNC_PTR func_ptr, 
                    const uint8_t readChainedFlag,
                    const uint8_t writeChainedFlag)
{
    return smb_protocol_execute(channel,
                                buffer_ptr,
                                smb_protocol,
                                writeCount,
                                pecEnable,
                                (MASTER_FUNC_PTR)func_ptr,
                                readChainedFlag,
                                writeChainedFlag);
}

uint8_t mctp_i2c_rx_register(const uint8_t channel, 
                            I2C_SLAVE_FUNC_PTR slaveFuncPtr)
{
   return smb_register_slave(channel, 
                             (SLAVE_FUNC_PTR)slaveFuncPtr);
   
}

void mctp_i2c_configure_and_enable(uint8_t channel, 
                                   uint16_t own_address, 
                                   uint8_t speed, 
                                   uint8_t port, 
                                   uint8_t configFlag)
{
    smbus_configure_and_enable(channel, 
                        own_address, 
                        speed, 
                        port, 
                        configFlag);
}

uint8_t mctp_i2c_get_chan_busy_status(uint8_t channel)
{
    return smb_busyStatus_get(channel);
}

uint16_t mctp_i2c_get_current_timestamp(void)
{
    return smb_get_current_timestamp();
}

void I2CSMB_GRP_InterruptHandler ( void )
{
    smb_isr();
}

void DMA_GRP_InterruptHandler (void)
{
    smb_dma_isr();
}

/*******************************************************************************
 End of File
 */
