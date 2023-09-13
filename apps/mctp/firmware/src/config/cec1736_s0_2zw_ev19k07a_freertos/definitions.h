/*******************************************************************************
  System Definitions

  File Name:
    definitions.h

  Summary:
    project system definitions.

  Description:
    This file contains the system-wide prototypes and definitions for a project.

 *******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2018 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *******************************************************************************/
//DOM-IGNORE-END

#ifndef DEFINITIONS_H
#define DEFINITIONS_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include "peripheral/i2c/plib_smb4_master_slave_common.h"
#include "peripheral/i2c/master/plib_smb4_master.h"
#include "peripheral/i2c/slave/plib_smb4_slave.h"
#include "peripheral/i2c/plib_smb3_master_slave_common.h"
#include "peripheral/i2c/master/plib_smb3_master.h"
#include "peripheral/i2c/slave/plib_smb3_slave.h"
#include "peripheral/i2c/plib_smb0_master_slave_common.h"
#include "peripheral/i2c/master/plib_smb0_master.h"
#include "peripheral/i2c/slave/plib_smb0_slave.h"
#include "peripheral/gpio/plib_gpio.h"
#include "peripheral/pcr/plib_pcr.h"
#include "peripheral/ecia/plib_ecia.h"
#include "peripheral/nvic/plib_nvic.h"
#include "peripheral/dma/plib_dma.h"
#include "peripheral/ec_reg_bank/plib_ec_reg_bank.h"
#include "peripheral/i2c/plib_smb2_master_slave_common.h"
#include "peripheral/i2c/master/plib_smb2_master.h"
#include "peripheral/i2c/slave/plib_smb2_slave.h"
#include "peripheral/i2c/plib_smb1_master_slave_common.h"
#include "peripheral/i2c/master/plib_smb1_master.h"
#include "peripheral/i2c/slave/plib_smb1_slave.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_smb_drv.h"



// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END

/* Device Information */
#define DEVICE_NAME			 "CEC1736_S0_2ZW"
#define DEVICE_ARCH			 "CORTEX-M4"
#define DEVICE_FAMILY		 "CEC173X"
#define DEVICE_SERIES		 "CEC"

/* CPU clock frequency */
#define CPU_CLOCK_FREQUENCY 96000000

// *****************************************************************************
// *****************************************************************************
// Section: System Functions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* System Initialization Function

  Function:
    void SYS_Initialize( void *data )

  Summary:
    Function that initializes all modules in the system.

  Description:
    This function initializes all modules in the system, including any drivers,
    services, middleware, and applications.

  Precondition:
    None.

  Parameters:
    data            - Pointer to the data structure containing any data
                      necessary to initialize the module. This pointer may
                      be null if no data is required and default initialization
                      is to be used.

  Returns:
    None.

  Example:
    <code>
    SYS_Initialize ( NULL );

    while ( true )
    {
        SYS_Tasks ( );
    }
    </code>

  Remarks:
    This function will only be called once, after system reset.
*/

void SYS_Initialize( void *data );

// *****************************************************************************
/* System Tasks Function

Function:
    void SYS_Tasks ( void );

Summary:
    Function that performs all polled system tasks.

Description:
    This function performs all polled system tasks by calling the state machine
    "tasks" functions for all polled modules in the system, including drivers,
    services, middleware and applications.

Precondition:
    The SYS_Initialize function must have been called and completed.

Parameters:
    None.

Returns:
    None.

Example:
    <code>
    SYS_Initialize ( NULL );

    while ( true )
    {
        SYS_Tasks ( );
    }
    </code>

Remarks:
    If the module is interrupt driven, the system will call this routine from
    an interrupt context.
*/

void SYS_Tasks ( void );


// *****************************************************************************
// *****************************************************************************
// Section: extern declarations
// *****************************************************************************
// *****************************************************************************




//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif /* DEFINITIONS_H */
/*******************************************************************************
 End of File
*/

