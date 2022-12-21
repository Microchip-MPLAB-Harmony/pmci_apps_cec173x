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
/** @file smb_slave_dispatch.hpp
 \brief the smb slave dispatch header file
 This file implements the slave dispatch layer. This file defines SMB_SLAVE_DISPATCH class

<b>Platform:</b> This is ARM-based component
  Details of hardware blocks on which this smbus reuse component is tested:
    smbus HW Block name - blk_i2c_spb, smbus HW Block build # - B0076   
    dma HW Block name - blk_dma_ahb, dma HW Block build # - B0007
    FPGA build used for testing -> MEC1322 FPGA build Rev-81-Randy - 081_chip_top_fpga2_run1
    //depotAE/projects/MEC1322/FPGA builds/Rev-81-Randy/081_chip_top_fpga2_run1.mcs

<b>Toolset:</b> MDK-Lite V 4.23; Armcc, Armasm, ArmLink, ArmAr, FromElf - V4.1.0.894 
<b>Reference:</b> smsc_reusable_fw_requirement.doc */

/*******************************************************************************
 $File: //depot_pcs/FWEng/solutions/cec1712_GEN2/maincodeline/secureboot_app/src/plib/smbus/smb_slave_dispatch.hpp $

Version Control Information (Perforce)
*******************************************************************************
$Revision: #2 $
$DateTime: 2020/01/06 09:12:42 $
$Author: I19961 $
  Change Description:
    3. Branched from //depotAE/ReUse_Repository/FW_ARC/driver/smbus/work_bench/source/  
    4. Updated file header for MEC1322 test environment
*******************************************************************************/
#ifndef SMB_SLAVE_DISPATCH_HPP_
#define SMB_SLAVE_DISPATCH_HPP_

#include "smb_config.h"
#include "smb.h"

#ifndef DISABLE_ALL_SLAVE_BUFFERS

namespace SMB{



class SMB_SLAVE_DISPATCH{

    enum BufferFreeStatus
    {
        BUFFER_FREE
        ,BUFFER_IN_USE
    };

    /******************************************************************************/
    /** struct SLAVE_APP_INFO \n
    * This structure is used to store information of a application that registers
    * for slave. The 'Usage' field is used to know whether the entry is available
    * or not to store application information.
    *******************************************************************************/
    typedef struct _SLAVE_APP_INFO_
    {
        SLAVE_FUNC_PTR      applFuncPtr;      /**< Application function pointer */
        BufferFreeStatus    usage:8;          /**< Usage flag - FREE or IN_USE */
    }SLAVE_APP_INFO;


    //private:
    public:

        /** Array of structures to store Application information */
        SLAVE_APP_INFO applicationInfo_[SMB_MAX_NUM_SLAVE_APP];  /* Coverity */

        SLAVE_APP_INFO* search_applicationInfo(SLAVE_FUNC_PTR slaveFuncPtr);
        SLAVE_APP_INFO* get_free_applicationInfo_entry(void);

        SMB_SLAVE_DISPATCH(const SMB_SLAVE_DISPATCH& r); //no copy
        SMB_SLAVE_DISPATCH& operator=(const SMB_SLAVE_DISPATCH& r);  //no assignment

    //public:

        SMB_SLAVE_DISPATCH(){};

        UINT8 register_app(SLAVE_FUNC_PTR slaveFuncPtr);
        UINT8 deregister_app(SLAVE_FUNC_PTR slaveFuncPtr);
        UINT8 inform_app(BUFFER_INFO *slave_buffer_info,const UINT8 slaveTransmitFlag)const;
    };

}
#endif //DISABLE_ALL_SLAVE_BUFFERS
#endif /* SMB_SLAVE_DISPATCH_HPP_ */
/** @}
*/
