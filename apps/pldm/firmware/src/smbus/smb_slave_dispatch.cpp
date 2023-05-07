/*****************************************************************************
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
*****************************************************************************/
/** @defgroup smb smb_slave_dispatch
 *  @{
 */
/** @file smb_slave_dispatch.cpp
 \brief the smb slave dispatch source file
 This file implements the slave dispatch layer. This file defines the functions
 declared in SMB_SLAVE_DISPATCH class

<b>Platform:</b> This is ARM-based component
  Details of hardware blocks on which this smbus reuse component is tested:
    smbus HW Block name - blk_i2c_spb, smbus HW Block build # - B0076   
    dma HW Block name - blk_dma_ahb, dma HW Block build # - B0007
    FPGA build used for testing -> MEC1322 FPGA build Rev-81-Randy - 081_chip_top_fpga2_run1
    //depotAE/projects/MEC1322/FPGA builds/Rev-81-Randy/081_chip_top_fpga2_run1.mcs

<b>Toolset:</b> MDK-Lite V 4.23; Armcc, Armasm, ArmLink, ArmAr, FromElf - V4.1.0.894 
<b>Reference:</b> smsc_reusable_fw_requirement.doc */

/*******************************************************************************
 $File: //depot_pcs/FWEng/solutions/cec1712_GEN2/maincodeline/secureboot_app/src/plib/smbus/smb_slave_dispatch.cpp $

Version Control Information (Perforce)
*******************************************************************************
$Revision: #1 $
$DateTime: 2019/07/31 22:19:26 $
$Author: vinayagp $
  Change Description:
    3. Branched from //depotAE/ReUse_Repository/FW_ARC/driver/smbus/work_bench/source/  
    4. Updated file header for MEC1322 test environment
*******************************************************************************/
extern "C"
{
    #include <common.h>
}
#include "smb_slave_dispatch.hpp"

#ifndef DISABLE_ALL_SLAVE_BUFFERS

using namespace SMB;

/******************************************************************************/
/** SMB_SLAVE_DISPATCH::register_app function.
 * This function registers a smbus slave application
 * @param slaveFuncPtr   The application function to call on receiving a packet
 * @return  STATUS_OK on successful registration, else error status
 * @note    Whenever a application expects data from smbus (i.e acts as slave) it
 *          needs to register using this function.
 *          The application function that is registered should only copy the
 *          packet from smbus buffer, it should not the process the data in that
 *          function
*******************************************************************************/
UINT8 SMB_SLAVE_DISPATCH::register_app(SLAVE_FUNC_PTR slaveFuncPtr)
{
    UINT8 retVal;

    //trace0(0, SMB_SLAVE_DISPATCH, 0, "register_app: Enter");

    if (search_applicationInfo(slaveFuncPtr)==NULLVOIDPTR)
    {
        /* Get a free SLAVE_AAPP_INFO structure */
        SLAVE_APP_INFO* const tempSlaveAppInfo = get_free_applicationInfo_entry();
        if (NULLVOIDPTR != tempSlaveAppInfo)
        {
            /* Store the application information */
            tempSlaveAppInfo->applFuncPtr = slaveFuncPtr;
            tempSlaveAppInfo->usage = BUFFER_IN_USE;
            retVal = (UINT8)STATUS_OK;
        }
        else
        {
            //trace0(0, SMB_SLAVE_DISPATCH, 0, "register_app: Maximum number of applications registered ");
            retVal = (UINT8)STATUS_MAX_APP_REGISTERED;
        }
    }
    else
    {
        //trace0(0, SMB_SLAVE_DISPATCH, 0, "register_app: Already registered ");
        retVal = (UINT8)STATUS_ALREADY_REGISTERED;
    }

    return retVal;

}/* SMB_SLAVE_DISPATCH::register_app */

/******************************************************************************/
/** SMB_SLAVE_DISPATCH::deregister_app function.
 * This function is used de-register a smbus slave application
 * @param   slaveFuncPtr   Application function pointer
 * @return  STATUS_OK on successful de-registration, else error status
*******************************************************************************/
UINT8 SMB_SLAVE_DISPATCH::deregister_app(SLAVE_FUNC_PTR slaveFuncPtr)
{
    UINT8 status;

    //trace0(0, SMB_SLAVE_DISPATCH, 0, "deregister_app: Enter");

    SLAVE_APP_INFO* const tempSlaveAppInfo = search_applicationInfo(slaveFuncPtr);
    if (NULLVOIDPTR == tempSlaveAppInfo)
    {
        //trace0(0, SMB_SLAVE_DISPATCH, 0, "deregister_app: smb_applicationFuncPntr_get: Application not found ");
        status = (UINT8)STATUS_APP_NOT_REGISTERED;
    }
    else
    {
        /* Free the Slave Application Info entry */
        tempSlaveAppInfo->applFuncPtr = (SLAVE_FUNC_PTR)NULLVOIDPTR;
        tempSlaveAppInfo->usage = BUFFER_FREE;
        status =  (UINT8)STATUS_OK;
    }

    return status;
}/* SMB_SLAVE_DISPATCH::deregister_app */

/******************************************************************************/
/** SMB_SLAVE_DISPATCH::inform_app function.
 * This function dispatches slave buffer to application
 * @param slave_buffer_info buffer information
 * @param slaveTransmitFlag flag to indicate if SLAVE TRANSMIT phase is pending
 * @return   STATUS_BUFFER_ERROR if no application has taken the data
 *           STATUS_BUFFER_DONE if any application has taken the data
 *           STATUS_BUFFER_NOT_DONE if any application wants the data but can't
 *           take the data currently
*******************************************************************************/
UINT8 SMB_SLAVE_DISPATCH::inform_app(BUFFER_INFO *slave_buffer_info, const UINT8 slaveTransmitFlag)const
{
    UINT8 status=(UINT8)STATUS_BUFFER_ERROR, i;

    //trace0(0, SMB_SLAVE_DISPATCH, 0, "inform_app: Enter");

    for (i=0;i<SMB_MAX_NUM_SLAVE_APP;i++)
    {
        /* Whenever a slave register's the entry in slaveAppInfo[][] is marked IN_USE.
         * So when we search for a registered slave we need to search only those entries
         * which are marked IN_USE.
         * This approach makes it easier for applications that register and de-register
         * whenever they want
         */
        if (BUFFER_IN_USE == applicationInfo_[i].usage)
        {
            if ((SLAVE_FUNC_PTR)NULLVOIDPTR != applicationInfo_[i].applFuncPtr)
            {
                status = applicationInfo_[i].applFuncPtr(slave_buffer_info, slaveTransmitFlag);
                if ((UINT8)STATUS_BUFFER_ERROR != status)
                {
                    /* If any application has identified the packet that it is meant for
                     * it, SMBus dispatch will stop searching for further applications */
                    //trace0(0, SMB_SLAVE_DISPATCH, 0, "inform_app: Application for packet found");
                    break; //  - 'for loop'
                }
            }
        }
    }

    return status;
}/* SMB_SLAVE_DISPATCH::inform_app */

/******************************************************************************/
/** SMB_SLAVE_DISPATCH::search_applicationInfo function.
 * Search for slave application registered according to command code
 * @param slaveFuncPtr   Application Function Pointer
 * @return   The pointer to SLAVE_ APP_INFO structure on success, NULL on failure
*******************************************************************************/
SMB_SLAVE_DISPATCH::SLAVE_APP_INFO* SMB_SLAVE_DISPATCH::search_applicationInfo(SLAVE_FUNC_PTR slaveFuncPtr)
{
    UINT8 i;

    //trace0(0, SMB_SLAVE_DISPATCH, 0, "search_applicationInfo: Enter");

    for (i=0;i<SMB_MAX_NUM_SLAVE_APP;i++)
    {
        /* Whenever a slave register's the entry in slaveAppInfo[][] is marked IN_USE.
         * So when we search for a registered slave we need to search only those entries
         * which are marked IN_USE.
         * This approach makes it easier for applications that register and de-register
         * whenever they want
         */
        if ((applicationInfo_[i].applFuncPtr == slaveFuncPtr)
             && (BUFFER_IN_USE == applicationInfo_[i].usage))

        {
            return &applicationInfo_[i];
        }
    }
    //trace0(0, SMB_SLAVE_DISPATCH, 0, "search_applicationInfo: Not found");
    return (SLAVE_APP_INFO*)NULLVOIDPTR;
}/* SMB_SLAVE_DISPATCH::search_applicationInfo */

/******************************************************************************/
/** SMB_SLAVE_DISPATCH::get_free_applicationInfo_entry function.
 * This function gets a free SLAVE_APP_INFO structure to store application information
 * @param None
 * @return The pointer to free SLAVE_ APP_INFO structure on success, NULL on failure
*******************************************************************************/
SMB_SLAVE_DISPATCH::SLAVE_APP_INFO* SMB_SLAVE_DISPATCH::get_free_applicationInfo_entry(void)
{
    UINT8 i;

    //trace0(0, SMB_SLAVE_DISPATCH, 0, "get_free_applicationInfo_entry: Enter");

    for (i=0;i<SMB_MAX_NUM_SLAVE_APP;i++)
    {
        if (BUFFER_FREE == applicationInfo_[i].usage)
        {
            return &applicationInfo_[i];
        }
    }

    //trace0(0, SMB_SLAVE_DISPATCH, 0, "get_free_applicationInfo_entry: No Free Slave App Info");
    return (SLAVE_APP_INFO*) NULLVOIDPTR;
} /* SMB_SLAVE_DISPATCH::get_free_applicationInfo_entry */
/** @}
*/
#endif //#ifndef DISABLE_ALL_SLAVE_BUFFERS
