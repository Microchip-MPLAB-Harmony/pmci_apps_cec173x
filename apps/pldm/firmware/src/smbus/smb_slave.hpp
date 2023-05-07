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
*****************************************************************************
 *  @{
 */
/** @file smb_slave.hpp
 \brief the smb slave header file
 This file defines the SMB_SLAVE class.

<b>Platform:</b> This is ARM-based component
  Details of hardware blocks on which this smbus reuse component is tested:
    smbus HW Block name - blk_i2c_spb, smbus HW Block build # - B0076   
    dma HW Block name - blk_dma_ahb, dma HW Block build # - B0007
    FPGA build used for testing -> MEC1322 FPGA build Rev-81-Randy - 081_chip_top_fpga2_run1
    //depotAE/projects/MEC1322/FPGA builds/Rev-81-Randy/081_chip_top_fpga2_run1.mcs

<b>Toolset:</b> MDK-Lite V 4.23; Armcc, Armasm, ArmLink, ArmAr, FromElf - V4.1.0.894 
<b>Reference:</b> smsc_reusable_fw_requirement.doc */

/*******************************************************************************
 $File: //depot_pcs/FWEng/solutions/cec1712_GEN2/maincodeline/secureboot_app/src/plib/smbus/smb_slave.hpp $

Version Control Information (Perforce)
*******************************************************************************
$Revision: #3 $
$DateTime: 2020/03/11 23:10:14 $
$Author: I19961 $
  Change Description:
    3. Branched from //depotAE/ReUse_Repository/FW_ARC/driver/smbus/work_bench/source/  
    4. Updated file header for MEC1322 test environment
*******************************************************************************/
#ifndef SMB_SLAVE_HPP_
#define SMB_SLAVE_HPP_

#include "smb_config.h"
#include "smb.h"
#include "smb_slave_dispatch.hpp"
#include "smb_slave_buffer.hpp"
#include "smb_hw.hpp"

namespace SMB{

#define READ_BIT            BIT_0_MASK

enum readcount_one
{
    READCOUNT_ONE   =   1
};

enum spare_buffer_size
{
    BUFFER_SIZE_SPARE = 8
};

/** Slave States */
enum SlaveStates
{
     SLAVE_DISABLED =0          /**< Slave Disabled */
    ,SLAVE_RESET_INITIALIZE     /**< Slave reset and intialize */
    ,SLAVE_IDLE                 /**< Slave in Idle state */
    ,SLAVE_NACK                 /**< Slave configured for NACK on 3rd byte */
    ,SLAVE_CLK_STRETCH          /**< Slave configured for clock stretch */
    ,SLAVE_ENABLE               /**< Slave configured to receive packet */
};

class SMB_SLAVE{

        //private:
        public:
        // HW_SMB&  hw_smb_;
        // DMA::HW_DMA&  hw_dma_;
        UINT8   spare_buffer_[BUFFER_SIZE_SPARE];
#ifndef DISABLE_ALL_SLAVE_BUFFERS
        SMB_SLAVE_BUFFER bufferMgmt_;
        SMB_SLAVE_DISPATCH dispatch_;
        BUFFER_INFO *active_buffer_;
        UINT8   state_;
        UINT8   nackMechanism_;
        bool    repeatReadFlag_;
        bool    slaveTxInProgressUsingDMA;
        bool    slaveRxDMAInterrupt;

        void get_data_and_transmit(void);       
        void inform_app_and_reconfigure(void);
  #ifndef DISABLE_PENDING_PACKETS       
        bool process_pending_packets(void);
        bool process_received_packets(void);
    #else
        void process_received_packets(void);
  #endif        
        void update_pec(void);
#else
        UINT8 ack_read_count_;
#endif
        UINT8 instance_;
        UINT8 dma_chan_;

        void configure(const UINT8 *const buffer_ptr,const UINT8 readCount,const bool configureForNackFlag,
                     bool dmaEnableFlag,const bool incrMemAddrFlag, const bool enableDmaInterrupt);
        void configure_isr(const UINT8 *const buffer_ptr,const UINT8 readCount,const bool configureForNackFlag,
                     bool dmaEnableFlag,const bool incrMemAddrFlag, const bool enableDmaInterrupt);
        //void proceed(const UINT8 *const buffer_ptr,const UINT8 writeCount,const bool pecFlag);
        void proceed(const UINT8 *const buffer_ptr,const UINT8 writeCount,const bool pecFlag, const bool slaveXmitDoneFlag);

        SMB_SLAVE(const SMB_SLAVE& r); //no copy
        SMB_SLAVE& operator=(const SMB_SLAVE& r);  //no assignment

        //public:

        SMB_SLAVE(UINT32 hw_smb_address, UINT32 hw_dma_address);
        SMB_SLAVE(void);
        void init(UINT8 instance_id, UINT8 *buffer_ptr, UINT8 buffer_size);
        void handle_bus_error(void);
        void enable(void);      
        void enable_isr(void);      
        bool isr(void);
#ifndef DISABLE_ALL_SLAVE_BUFFERS
        void dma_isr(void);    
        bool event_task(void);
    #ifndef DISABLE_PENDING_PACKETS 
        bool interrupt_task(void);
        bool check_and_process_pending_pkts(void);
        bool check_and_process_recvd_pkts(void);
        bool disable(void);
    #else
    void check_and_process_recvd_pkts(void);
    void interrupt_task(void);
        void disable(void);
    #endif  
        
        void free_active_buffer(void);

        /******************************************************************************/
        /** register_app function.
         * This function registers a slave application
         * @param slaveFuncPtr   The application function to call on receiving a packet
         * @return  STATUS_OK on successful registration, else error status
        *******************************************************************************/
        UINT8 register_app(SLAVE_FUNC_PTR slaveFuncPtr)
        {
            return dispatch_.register_app(slaveFuncPtr);
        }/* register_app */

        /******************************************************************************/
        /** deregister_app function.
         * This function is used de-register a slave application
         * @param   slaveFuncPtr   Application function pointer
         * @return  STATUS_OK on successful de-registration, else error status
        *******************************************************************************/
        UINT8 deregister_app(SLAVE_FUNC_PTR slaveFuncPtr)
        {
            return dispatch_.deregister_app(slaveFuncPtr);
        }/* deregister_app */

        /******************************************************************************/
        /** update_nack_mechanism function.
         * This function updates the nack mechanism for slave
         * @param   nackMechanism
         * @return  None
        *******************************************************************************/
        void update_nack_mechanism(const UINT8 nackMechanism)
        {
            nackMechanism_ = nackMechanism;
        }/* update_nack_mechanism */

#endif
    } ;
}

#endif /* SMB_SLAVE_HPP_ */
/** @}
*/
