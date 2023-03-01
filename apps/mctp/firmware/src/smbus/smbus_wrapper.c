/******************************************************************************
* © 2018 Microchip Technology Inc. and its subsidiaries.
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
*******************************************************************************
*$File: //depot_pcs/FWEng/solutions/Glacier_GEN3/maincodeline/Harmony/pmci_stack/pmci_apps_cec173x/apps/mctp/firmware/src/smbus/smbus_wrapper.c $

Version Control Information (Perforce)
*******************************************************************************
$Revision: #3 $
$DateTime: 2023/02/24 06:07:07 $
$Author: i64652 $
Change Description: 
    1. Initial Draft 
    2. Added support for slave for host side
    3. Updated for proper traces and error conditions
    4. Updated smb_slave_callback_function for SET_REPORT_FEATURE command
*******************************************************************************/
/** @file smbus_app.c
* \brief SMBus Application source file
* \author consultant_jvasanth
* 
* This file implements the smbus wrapper to support sensor r/w functions
******************************************************************************/
#include "common.h"
#include "smb.h"


#define SMB_APP_TX_BUFFER_SIZE  128

UINT8 master_callback(UINT8 status, UINT8 *buffer_ptr, SMB_MAPP_CBK_NEW_TX *newTxParams);

UINT8 smbAppTxBuffer[SMB_APP_TX_BUFFER_SIZE];
UINT8 g_ret_status;

/******************************************************************************/
/**     Master Callback Function
 *      This function is called once the master transaction is complete
 *       Here we update the g_ret_status variable
* @param status status returned from smbus layer
* @param buffer_ptr pointer to buffer
* @return APP_RETVAL_RELEASE_SMBUS for releasing smbus
*******************************************************************************/
UINT8 master_callback(UINT8 status, UINT8 *buffer_ptr, SMB_MAPP_CBK_NEW_TX *newTxParams)
{
    UINT8 retVal=APP_RETVAL_RELEASE_SMBUS;    

    //trace1(0, SMB_WRAP, 0, "master_callback: status = %02Xh", status);

    switch (status)
    {
    case SUCCESS_TX:
        /* This status will be returned on successful write protocols:
         * SMB_SEND_BYTE, SMB_WRITE_BYTE, SMB_WRITE_WORD, SMB_WRITE_BLOCK */

  //      TRACE0(354, SMB_WRAP, 0, "master_callback: SUCCESS_TX");
        g_ret_status = 0;

        break;

    case ERROR_LAB:         // Lost Arbitration 
    case ERROR_MADDR_NAKX:  // Address NACK
    case ERROR_MDATA_NAKX:  // DATA NACK 
    case ERROR_PEC:         // PEC Errors

  //      TRACE0(355, SMB_WRAP, 0, "master_callback: Protocol Error");        

        break;

    case ERROR_SMB_DISABLED:        //SMBus firmware disabled (by host)
    case ERROR_CLK_DATA_NOT_HIGH:   //SMBus CLK & DATA lines not high
    case ERROR_BER_TIMEOUT:         //Bus Error due to timeouts
    case ERROR_BER_NON_TIMEOUT:     //Bus Error due to non timeouts (e.g. invalid START/STOP conditions)

  //      TRACE0(356, SMB_WRAP, 0, "master_callback: SMBus BER or Not Ready ");

        break;

    case SUCCESS_RX:
        /* This status will be returned on successful receive protocols:
         * SMB_RECEIVE_BYTE, SMB_READ_BYTE, SMB_READ_WORD, SMB_READ_BLOCK */ 
        /* For SMB_RECEIVE_BYTE protocol buffer_ptr[1] will contain the received byte */ 
        /* For SMB_READ_BYTE protocol buffer_ptr[3] will contain the received byte */ 
        /* For SMB_READ_WORD protocol buffer_ptr[3] & buffer_ptr[4] will contain the received word */ 
        /* For SMB_READ_BLOCK protocol buffer_ptr[3] will contain the read block data length */ 

    //  TRACE0(357, SMB_WRAP, 0, "master_callback: SUCCESS_RX");

        g_ret_status = 1;


        break;

    default:
        trace0(0, SMB_WRAP, 0, "master_callback: Invalid status");
    }

    return retVal;

}/*end smbApp_master_callback() */


/******************************************************************************/
/**     smb_callback \n\n
*       This function is the callback function which is invoked by smbus driver
*  at various instances for notifications
* @param channel channel
* @param eventType the type of event for notification
* @param eventValue parameter for the notification, if any
* @return None
* @note Note that only one application should have this callback.
*******************************************************************************/
VOID smb_callback(const UINT8 channel,const UINT8 eventType,const UINT8 eventValue)
{
    //UINT8 port;

    switch (eventType)
    {
    case SMB_CBK_DISABLED:      
        //smbus controller is disabled
    //  TRACE0(359, SMB_WRAP, 0, "smb_callback: SMB_CBK_DISABLED");
        break;

    case SMB_CBK_HW_ENABLED:        
        //smbus controller is enabled
    //  TRACE0(360, SMB_WRAP, 0, "smb_callback: SMB_CBK_HW_ENABLED");       
        break;

    case SMB_CBK_BER:
        //smbus controller Bus Error
    //  TRACE0(361, SMB_WRAP, 0, "smb_callback: SMB_CBK_BER");
        break;

    case SMB_CBK_BUSY:      
        //smbus controller Busy
    //  TRACE0(362, SMB_WRAP, 0, "smb_callback: SMB_CBK_BUSY");
        break;

    case SMB_CBK_PORT_ERROR_SET:        
        //smbus controller Port Error
        //port = eventValue;        
    //  TRACE0(363, SMB_WRAP, 0, "smb_callback: SMB_CBK_PORT_ERROR_SET");
        break;

    case SMB_CBK_PORT_ERROR_CLR:
        //smbus controller Port Error Cleared
        //port = eventValue;        
    //  TRACE0(364, SMB_WRAP, 0, "smb_callback: SMB_CBK_PORT_ERROR_CLR");
        break;

    default:
    //  TRACE0(365, SMB_WRAP, 0, "smb_callback: default");
        break;
    }
}
