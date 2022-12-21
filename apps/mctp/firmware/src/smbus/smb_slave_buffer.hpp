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
/** @defgroup smb smb_slave_buffer
 *  @{
 */
/** @file smb_slave_buffer.hpp
 \brief the smb slave buffer header file
 This file defines the SMB_SLAVE_BUFFER class

<b>Platform:</b> This is ARM-based component
  Details of hardware blocks on which this smbus reuse component is tested:
    smbus HW Block name - blk_i2c_spb, smbus HW Block build # - B0076   
    dma HW Block name - blk_dma_ahb, dma HW Block build # - B0007
    FPGA build used for testing -> MEC1322 FPGA build Rev-81-Randy - 081_chip_top_fpga2_run1
    //depotAE/projects/MEC1322/FPGA builds/Rev-81-Randy/081_chip_top_fpga2_run1.mcs

<b>Toolset:</b> MDK-Lite V 4.23; Armcc, Armasm, ArmLink, ArmAr, FromElf - V4.1.0.894 
<b>Reference:</b> smsc_reusable_fw_requirement.doc */

/*******************************************************************************
 $File: //depot_pcs/FWEng/solutions/cec1712_GEN2/maincodeline/secureboot_app/src/plib/smbus/smb_slave_buffer.hpp $

Version Control Information (Perforce)
*******************************************************************************
$Revision: #2 $
$DateTime: 2020/01/06 09:12:42 $
$Author: I19961 $
  Change Description:
    3. Branched from //depotAE/ReUse_Repository/FW_ARC/driver/smbus/work_bench/source/  
    4. Updated file header for MEC1322 test environment
*******************************************************************************/
#ifndef SMB_SLAVE_BUFFER_HPP_
#define SMB_SLAVE_BUFFER_HPP_

#include "smb_config.h"
#include "smb.h"

#ifndef DISABLE_ALL_SLAVE_BUFFERS

namespace SMB{

class SMB_SLAVE_BUFFER{

    //private:
    public:

        UINT8 *ptr_bufferPool_;                                 /* Coverity */
        BUFFER_INFO buffer_info_[SMB_MAX_NUM_SLAVE_BUFFER];
        UINT8 total_count_; /* Total Buffers Count */
        UINT8 size_;   /* Buffer Size */

        /** List of free buffers, implemented as a stack */
        BUFFER_INFO *free_list_info_[SMB_MAX_NUM_SLAVE_BUFFER];
        UINT8 top_index_free_list_; /** Top Index of Free Buffer L ist (Stack Top) */

        /** List of received buffers, implemented as a queue */
        BUFFER_INFO *received_list_info_[SMB_MAX_NUM_SLAVE_BUFFER];
        UINT8 front_index_received_list_; /** Front index of Received Buffer List */
        UINT8 count_received_list_; /** Number of entries in Received Buffer List */

    #ifndef DISABLE_PENDING_PACKETS 
        BUFFER_INFO *pending_list_info_[SMB_MAX_NUM_SLAVE_BUFFER];
        UINT8 front_index_pending_list_;
        UINT8 count_pending_list_;
    #endif

    
        void allocate(const UINT8 NumOfBuffers,  UINT8 **ptr_buffers);
        void listIndexes_init(void);

        SMB_SLAVE_BUFFER(const SMB_SLAVE_BUFFER& r); //no copy
        SMB_SLAVE_BUFFER& operator=(const SMB_SLAVE_BUFFER& r);  //no assignment

    //public:

        SMB_SLAVE_BUFFER (){};

        void init(UINT8 *buffer_ptr, const UINT8 buffer_size);
        BUFFER_INFO* free_buffer_get(void);
        BUFFER_INFO* received_buffer_get(void);     
        void free_buffer_put(BUFFER_INFO *ptr_buffer_info);
        void recvd_buffer_put(BUFFER_INFO *ptr_buffer_info);
    #ifndef DISABLE_PENDING_PACKETS 
        void pending_buffer_put(BUFFER_INFO *ptr_buffer_info);
        BUFFER_INFO* pending_buffer_get(void);
        bool is_timestamp_expired(const UINT16 TimeStamp,const UINT16 checkInterval) const;
    #endif
        

        /******************************************************************************/
        /** is_there_any_free_buffer function.
         * This function checks if there is any buffer available in the free list
         * @param None
         * @return TRUE/FALSE - TRUE if free buffer available, else FALSE
        *******************************************************************************/
        bool is_there_any_free_buffer(void) const
        {
            return (top_index_free_list_ != 0);

        }/* is_there_any_free_buffer */


        /******************************************************************************/
        /** is_there_any_received_packet function.
         * This function checks if any packet is available in the received list
         * @param None
         * @return TRUE/FALSE - TRUE if the packet is available, else FALSE
        *******************************************************************************/
        bool is_there_any_received_packet(void) const
        {
            return (count_received_list_ != 0);

        }/* is_there_any_received_packet */

    #ifndef DISABLE_PENDING_PACKETS 
        /******************************************************************************/
        /** is_there_any_pending_packet function
         * This function checks if any packet is available in the pending list
         * @param None
         * @return TRUE/FALSE - TRUE if the packet is available, else FALSE
        *******************************************************************************/
        bool is_there_any_pending_packet(void) const
        {
            return (count_pending_list_ != 0);

        }/* is_there_any_pending_packet */

        /******************************************************************************/
        /** get_count_pendingBufferList function
         * This function gets the count of packets in pending buffer list
         * @param None
         * @return count
        *******************************************************************************/
        UINT8 get_count_pendingBufferList(void) const
        {
            return count_pending_list_;

        }/* get_count_pendingBufferList */
    #endif

        /******************************************************************************/
        /** get_count_receivedBufferList function
         * This function gets the count of packets in received buffer list
         * @param None
         * @return count
        *******************************************************************************/
        UINT8 get_count_receivedBufferList(void) const
        {
            return count_received_list_;

        }/* get_count_receivedBufferList */

        /******************************************************************************/
        /** get_buffers_count function.
         * This function returns the total number of buffers available
         * @param None
         * @return count
        *******************************************************************************/
        UINT8 get_buffers_count(void) const
        {
            return total_count_;
        }/* get_buffers_count */

        /******************************************************************************/
        /** get_buffer_size function
         * This function returns the buffer size
         * @param None
         * @return buffer_size
        *******************************************************************************/
        UINT8 get_buffer_size(void) const
        {
            return size_;
        }/* get_buffer_size */


    };
}
#endif //#ifndef DISABLE_ALL_SLAVE_BUFFERS
#endif /* SMB_SLAVE_BUFFER_HPP_ */
/** @}
*/
