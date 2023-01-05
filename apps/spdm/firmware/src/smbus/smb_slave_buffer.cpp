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
/** @file smb_slave_buffer.cpp
 \brief the smb slave buffer source file
 This file implements the buffer management for slave. It provides the definitions of
 function declared in SMB_SLAVE_BUFFER class

<b>Platform:</b> This is ARM-based component
  Details of hardware blocks on which this smbus reuse component is tested:
    smbus HW Block name - blk_i2c_spb, smbus HW Block build # - B0076   
    dma HW Block name - blk_dma_ahb, dma HW Block build # - B0007
    FPGA build used for testing -> MEC1322 FPGA build Rev-81-Randy - 081_chip_top_fpga2_run1
    //depotAE/projects/MEC1322/FPGA builds/Rev-81-Randy/081_chip_top_fpga2_run1.mcs

<b>Toolset:</b> MDK-Lite V 4.23; Armcc, Armasm, ArmLink, ArmAr, FromElf - V4.1.0.894 
<b>Reference:</b> smsc_reusable_fw_requirement.doc */

/*******************************************************************************
 $File: //depot_pcs/FWEng/solutions/cec1712_GEN2/maincodeline/secureboot_app/src/plib/smbus/smb_slave_buffer.cpp $

Version Control Information (Perforce)
*******************************************************************************
$Revision: #1 $
$DateTime: 2019/07/31 22:19:26 $
$Author: vinayagp $
  Change Description:
    3. Branched from //depotAE/ReUse_Repository/FW_ARC/driver/smbus/work_bench/source/  
    4. Temporary update for unit testing purpose
    5. Updated file header for MEC1322 test environment
*******************************************************************************/
extern "C"
{
    #include <common.h>
}

#include "smb_slave_buffer.hpp"

#ifndef DISABLE_ALL_SLAVE_BUFFERS
using namespace SMB;

/******************************************************************************/
/** SMB_SLAVE_BUFFER::init function
 * This function should be called during initialization to setup the slave buffers
 * @param buffer_ptr pointer to a allocated buffer
 * @return None
 *******************************************************************************/
void SMB_SLAVE_BUFFER::init(UINT8 *buffer_ptr, const UINT8 buffer_size)
{
    UINT8 *ptr_buffer[SMB_MAX_NUM_SLAVE_BUFFER];
    UINT8 BufferIndexInPool, i;

    //trace1(0, SMB_SLAVE_BUFFER, 0, "init: Enter ch %d", instance+1);

    /* Initialize the buffer(free, received, pending) list indexes  */
    listIndexes_init();

    ptr_bufferPool_ = buffer_ptr;
    size_ = buffer_size;

    //trace1(0, SMB_SLAVE_BUFFER, 0, "init: bufferPool_ = %04Xh ", (UINT16)(UINT32)&ptr_bufferPool_[0]);

    for (i=0; i<SMB_MAX_NUM_SLAVE_BUFFER; i++)
    {
        BufferIndexInPool = (UINT8)(size_*i);
        ptr_buffer[i] = &ptr_bufferPool_[BufferIndexInPool];
        //trace1(0, SMB_SLAVE_BUFFER, 0, "init: ptr_buffer = %04Xh ", (UINT16)(UINT32)ptr_buffer[i]);
    }

    /* Modified the slave buffer size to 4 less than actual to account for, extra bytes
     * received by slave even when configured for less. See bug #11927 */
    size_ = size_ - 4;
    allocate(SMB_MAX_NUM_SLAVE_BUFFER, ptr_buffer);

}/* SMB_SLAVE_BUFFER::init */

/******************************************************************************/
/** SMB_SLAVE_BUFFER::allocate function
 * This function allocates the slave buffers
* @param NumOfBuffers   Number of buffers passed
* @param ptr_buffers    Address of buffer pointer array
* @return   OK on success, ERROR on failure
*******************************************************************************/
void SMB_SLAVE_BUFFER::allocate(const UINT8 NumOfBuffers,  UINT8 **ptr_buffers)
{
    UINT8 i;

    //trace1(0, SMB_SLAVE_BUFFER, 0, "allocate:  NumOfBuffers = %02Xh",NumOfBuffers );

    /* Store the buffer information */
    for (i=0;i<NumOfBuffers;i++)
    {
        buffer_info_[i].buffer_ptr = ptr_buffers[i];
        buffer_info_[i].TimeStamp = 0x0;

        free_list_info_[i] = &buffer_info_[i];

        //trace2(0, SMB_SLAVE_BUFFER, 0, "init: &free_list_info_[%d] = %04Xh ", i, (UINT16)(UINT32)free_list_info_[i]);
    }
    top_index_free_list_ = NumOfBuffers;

    /* Store the buffer size and number of buffers */
    total_count_ = NumOfBuffers;    

}/* SMB_SLAVE_BUFFER::get_buffer_size */

/******************************************************************************/
/** SMB_SLAVE_BUFFER::listIndexes_init function.
 * This function initializes the slave buffers' indexes
 * @param None
 * @return None
*******************************************************************************/
void SMB_SLAVE_BUFFER::listIndexes_init(void)
{
    //trace0(0, SMB_SLAVE_BUFFER, 0, "listIndexes_init: Enter");

    top_index_free_list_ = 0;
#ifndef DISABLE_PENDING_PACKETS     
    front_index_pending_list_ = 0;
    count_pending_list_ = 0;
#endif  
    front_index_received_list_ = 0;
    count_received_list_ = 0;

}/* SMB_SLAVE_BUFFER::listIndexes_init */


/******************************************************************************/
/** SMB_SLAVE_BUFFER::free_buffer_get function.
 * This function returns a buffer from the free buffers List
 * @param   None
 * @return  Pointer to free buffer info structure if available, else NULL
*******************************************************************************/
BUFFER_INFO* SMB_SLAVE_BUFFER::free_buffer_get(void)
{
    if (0 == top_index_free_list_)
    {
        //trace0(0, SMB_SLAVE_BUFFER, 0, "free_buffer_get: No Free Buffer");
        return (BUFFER_INFO*)NULLVOIDPTR;
    }
    /* Get the element from stack top and decrement stack top index */
    --top_index_free_list_;

    //trace1(0, SMB_SLAVE_BUFFER, 0, "free_buffer_get: buffer_ptr = %04Xh",(UINT16)(UINT32)free_list_info_[(top_index_free_list_)]->buffer_ptr);
    return free_list_info_[(top_index_free_list_)];

}/* SMB_SLAVE_BUFFER::free_buffer_get */

/******************************************************************************/
/** SMB_SLAVE_BUFFER::received_buffer_get function.
 * This function returns a buffer from the received buffers' List
 * @param   None
 * @return   Pointer to received buffer info structure if available, else NULL
*******************************************************************************/
BUFFER_INFO* SMB_SLAVE_BUFFER::received_buffer_get(void)
{
    BUFFER_INFO *buffer_info_pntr;

    if (0 == count_received_list_)
    {
        //trace0(0, SMB_SLAVE_BUFFER, 0, "received_buffer_get: No buffer in pending list ");
        return (BUFFER_INFO*)NULLVOIDPTR;
    }

    /* Get the element from queue front */
    buffer_info_pntr = received_list_info_[front_index_received_list_];

    /* Shift queue front index */
    ++front_index_received_list_;
    front_index_received_list_ %= total_count_;

    /* Decrement queue count */
    --count_received_list_;

    //trace1(0, SMB_SLAVE_BUFFER, 0, "received_buffer_get: buffer_ptr return = %04Xh",(UINT16)(UINT32)buffer_info_pntr->buffer_ptr);

    return buffer_info_pntr;

}/* SMB_SLAVE_BUFFER::received_buffer_get */

/******************************************************************************/
/** SMB_SLAVE_BUFFER::pending_buffer_get function.
 * This function returns a buffer from the pending buffers' list
 * @param   None
 * @return   Pointer to pending buffer info structure if available, else NULL
*******************************************************************************/
#ifndef DISABLE_PENDING_PACKETS 
BUFFER_INFO* SMB_SLAVE_BUFFER::pending_buffer_get(void)
{
    BUFFER_INFO *buffer_info_pntr;

    if (0 == count_pending_list_)
    {
        //trace0(0, SMB_SLAVE_BUFFER, 0, "pending_buffer_get: No buffer in pending list ");
        return (BUFFER_INFO*)NULLVOIDPTR;
    }

    /* Get the element from queue front */
    buffer_info_pntr = pending_list_info_[front_index_pending_list_];

    /* Shift queue front index */
    ++front_index_pending_list_;
    front_index_pending_list_ %= total_count_;

    //trace1(0, SMB_SLAVE_BUFFER, 0, "pending_buffer_get: buffer_ptr = %04Xh",(UINT16)(UINT32)buffer_info_pntr->buffer_ptr);

    /* Decrement queue count */
    --count_pending_list_;

    return buffer_info_pntr;

}/* SMB_SLAVE_BUFFER::pending_buffer_get */
#endif

/******************************************************************************/
/** SMB_SLAVE_BUFFER::free_buffer_put function.
 * This function puts a buffer into free buffers List
 * @param   ptr_buffer_info  pointer to BUFFER_INFO structure
 * @return  None
*******************************************************************************/
void SMB_SLAVE_BUFFER::free_buffer_put(BUFFER_INFO *ptr_buffer_info)
{
    if (total_count_ == top_index_free_list_)
    {
        //trace0(0, SMB_SLAVE_BUFFER_ERR, 0, "free_buffer_put: No Free Buffer Slot, Something wrong ");
        return;
    }
    ptr_buffer_info->TimeStamp = 0x0;

    //trace1(0, SMB_SLAVE_BUFFER, 0, "free_buffer_put: buffer_ptr = %04Xh",(UINT16)(UINT32)ptr_buffer_info->buffer_ptr);

    /* Put the element into stack and increment stack top index */
    free_list_info_[(top_index_free_list_)] = ptr_buffer_info;
    ++top_index_free_list_;

}/* SMB_SLAVE_BUFFER::free_buffer_put */

/******************************************************************************/
/** SMB_SLAVE_BUFFER::recvd_buffer_put function.
 * This function puts a buffer into received buffers List
 * @param    ptr_buffer_info  pointer to BUFFER_INFO structure
 * @return  None
*******************************************************************************/
void SMB_SLAVE_BUFFER::recvd_buffer_put(BUFFER_INFO *ptr_buffer_info)
{
    UINT32 newElementIndex;

    if (count_received_list_ >= total_count_)
    {
        //trace0(0, SMB_SLAVE_BUFFER_ERR, 0, "recvd_buffer_put: No Free Buffer Slot in Pending List, Something wrong ");
        return;
    }

    /* Calculate the position of the new element in the queue */
    newElementIndex = (count_received_list_ + front_index_received_list_) % total_count_;

    //trace1(0, SMB_SLAVE_BUFFER, 0, "recvd_buffer_put: buffer_ptr = %04Xh",(UINT16)(UINT32)ptr_buffer_info->buffer_ptr);

    /* Add the new element to the queue */
    received_list_info_[newElementIndex] = ptr_buffer_info;

    /* Increment queue count */
    ++count_received_list_;

}/* SMB_SLAVE_BUFFER::recvd_buffer_put */

/******************************************************************************/
/** SMB_SLAVE_BUFFER::pending_buffer_put function.
 * This function puts a buffer into pending buffers List
 * @param   ptr_buffer_info    Pointer to BUFFER_INFO structure
 * @return  None
*******************************************************************************/
#ifndef DISABLE_PENDING_PACKETS 
void SMB_SLAVE_BUFFER::pending_buffer_put(BUFFER_INFO *ptr_buffer_info)
{
    UINT32 newElementIndex;

    if (count_pending_list_ >= total_count_)
    {
        //TRACE0(58, SMB_SLAVE_BUFFER_ERR, 0, "pending_buffer_put: No Free Buffer Slot in Pending List, Something wrong ");
        return;
    }

    /* Calculate the position of the new element in the queue */
    newElementIndex = (count_pending_list_ + front_index_pending_list_) % total_count_;

    //trace1(0, SMB_SLAVE_BUFFER, 0, "pending_buffer_put: buffer_ptr = %04Xh",(UINT16)(UINT32)ptr_buffer_info->buffer_ptr);

    /* Add the new element to the queue */
    pending_list_info_[newElementIndex] = ptr_buffer_info;

    /* Increment queue count */
    ++count_pending_list_;
    return;
}/* SMB_SLAVE_BUFFER::pending_buffer_put */

/******************************************************************************/
/** SMB_SLAVE_BUFFER::is_timestamp_expired function.
 * This function checks if the timestamp has expired
 * @param TimeStamp  Time Stamp
 * @param checkInterval Interval to check for expiration
 * @return TRUE/FALSE -  TRUE if timestamp has expired, else FALSE
*******************************************************************************/
bool SMB_SLAVE_BUFFER::is_timestamp_expired(const UINT16 TimeStamp,const UINT16 checkInterval) const
{
     UINT16 EndTime, TimeInterval;

     //trace1(0, SMB_SLAVE_BUFFER, 0, "is_timestamp_expired: Enter, TimeStamp =  %04Xh", TimeStamp);

     EndTime = (UINT16)(kGET_TICKS());

     //trace1(0, SMB_SLAVE_BUFFER, 0, "is_timestamp_expired: EndTime =  %04Xh", EndTime);

     if (EndTime >= TimeStamp)
     {
          TimeInterval = EndTime - TimeStamp;
     }
     else
     {
          TimeInterval = (0xFFFF - TimeStamp) + EndTime;
     }

     //trace1(0, SMB_SLAVE_BUFFER, 0, "is_timestamp_expired: TimeInterval = %04Xh", TimeInterval);

     if (TimeInterval >= checkInterval)
     {
          return true;
     }

     return false;

}/* End SMB_SLAVE_BUFFER::is_timestamp_expired() */
#endif

/** @}
*/
#endif //DISABLE_ALL_SLAVE_BUFFERS
