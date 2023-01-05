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
/** @defgroup smb smb_master
 *  @{
 */
/** @file smb_master.hpp
 \brief the smb master header file
 This file defines the SMB_MASTER class

<b>Platform:</b> This is ARM-based component
  Details of hardware blocks on which this smbus reuse component is tested:
    smbus HW Block name - blk_i2c_spb, smbus HW Block build # - B0076   
    dma HW Block name - blk_dma_ahb, dma HW Block build # - B0007
    FPGA build used for testing -> MEC1322 FPGA build Rev-81-Randy - 081_chip_top_fpga2_run1
    //depotAE/projects/MEC1322/FPGA builds/Rev-81-Randy/081_chip_top_fpga2_run1.mcs

<b>Toolset:</b> MDK-Lite V 4.23; Armcc, Armasm, ArmLink, ArmAr, FromElf - V4.1.0.894 
<b>Reference:</b> smsc_reusable_fw_requirement.doc */

/*******************************************************************************
 $File: //depot_pcs/FWEng/solutions/cec1712_GEN2/maincodeline/secureboot_app/src/plib/smbus/smb_master.hpp $

Version Control Information (Perforce)
*******************************************************************************
$Revision: #2 $
$DateTime: 2020/01/06 09:12:42 $
$Author: I19961 $
  Change Description:
    2. Code changes to support Master Tx chaining
    1. Branched from //depot_pcs/FWEng/projects/MEC1324/maincodeline/libSmbus/source/smb/ 
        label - MEC1324_LIB_SMBUS_0400
*******************************************************************************/
#ifndef SMB_MASTER_HPP_
#define SMB_MASTER_HPP_

#include "smb_hw.hpp"
#include "smb.h"
#include "smb_config.h"

#ifndef DISABLE_SMB_MASTER

namespace SMB{

/** Master States */
enum MasterStates
{
    MASTER_DISABLED =0          /**< Master Disabled */
    ,MASTER_RESET_INITIALIZE    /**< Master reset and initialize */
    ,MASTER_IDLE                /**< Master in idle state */
    ,MASTER_ENABLE              /**< Master performing a transaction */
    ,MASTER_DONE                    /**< Master completed a transaction */
};

class SMB_MASTER{

  //private:
    public:

    SMB_MAPP_CBK_NEW_TX appCbkNewTx_;
    MASTER_FUNC_PTR ptr_func_;
    // HW_SMB&  hw_smb_;
    // DMA::HW_DMA&  hw_dma_;
    UINT8   *ptr_buffer_;
    UINT32  masterCommand_;
    VUINT8  state_;
    UINT8   complStat_;
    UINT8   dmaEndAddrOffset_;
    UINT8   dmaEndAddrOffsetSave_;
    UINT8   wdtInterval_;
    UINT8   wdtCounter_;
    UINT8   appRetVal_;
    UINT8   instance_;
    UINT8   dma_chan_;
    bool    checkPEC_;
    bool    readBlkFlag_;
    bool    readChainedFlag_;
    bool    writeChainedFlag_;    

    SMB_MASTER(const SMB_MASTER& r); //no copy
    SMB_MASTER& operator=(const SMB_MASTER& r);  //no assignment

    void proceed(void);
    bool is_master_address_nack(void);
    bool is_PEC_valid(void) const;

  //public:

    SMB_MASTER(UINT32 hw_smb_address, UINT32 hw_dma_address);
    SMB_MASTER(void);

    void init(const UINT8 instance_id);
    void disable(void);
    void enable(void);
    void interrupt_task(void);
    void setup(UINT8 *buffer_ptr, bool pecEnable, MASTER_FUNC_PTR func_ptr);
    void form_cmd_value(const UINT8 smb_protocol, UINT8 writeCount, 
         const UINT8 i2c_readCount, const bool readChainedFlag, const bool wrChainedFlag);
    void xmit(void);
    void receive(UINT8 readCount);
    void xmit_chained(UINT8 writeCount);
    void retry(void);
    void handle_bus_error(void);
    UINT8 inform_app(void);
    bool isr(void);
    void ber_isr(void);
    UINT8 check_busy(void) const;
    UINT8 check_busy(MASTER_FUNC_PTR app_func_ptr) const;
    bool is_ready_for_port_change(void) const;
    bool is_wdt_expired(void);
    void dma_isr(void); 

    /******************************************************************************/
    /** get_state function.
     * This function returns the master state
     * @return state - master state
     * @param None
    *******************************************************************************/
    UINT8 get_state(void) const
    {
        return state_;
    }/* get_state */

    void update_wdt_interval(const UINT8 wdt_interval_value)
    {
        wdtInterval_ = wdt_interval_value;
    }/* is_master_idle */

    /******************************************************************************/
    /** set_completion_status function.
     * This function updates the master completion status
     * @return None
     * @param None
    *******************************************************************************/
    void set_completion_status(const UINT8 status)
    {
        complStat_ = status;
    }/* set_completion_status */

    /******************************************************************************/
    /** is_master_idle function.
     * This function checks if master state is idle
     * @return TRUE/FALSE
     * @param None
    *******************************************************************************/
    bool is_master_idle(void) const
    {
        return (MASTER_IDLE == state_);

    }/* is_master_idle */   

}; /* class SMB_MASTER */

}/* namespace SMB */

#endif //DISABLE_SMB_MASTER

#endif /* SMB_MASTER_HPP_ */
/** @}
*/
