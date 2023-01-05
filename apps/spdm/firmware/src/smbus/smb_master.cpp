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
/** @defgroup smb smb_master
 *  @{
 */
/** @file smb_master.cpp
 \brief the smb master cpp file
 This file implements the smb master functionality. It provides the definitions of
 function declared in SMB_MASTER class

<b>Platform:</b> This is ARM-based component
  Details of hardware blocks on which this smbus reuse component is tested:
    smbus HW Block name - blk_i2c_spb, smbus HW Block build # - B0076   
    dma HW Block name - blk_dma_ahb, dma HW Block build # - B0007
    FPGA build used for testing -> MEC1322 FPGA build Rev-81-Randy - 081_chip_top_fpga2_run1
    //depotAE/projects/MEC1322/FPGA builds/Rev-81-Randy/081_chip_top_fpga2_run1.mcs

<b>Toolset:</b> MDK-Lite V 4.23; Armcc, Armasm, ArmLink, ArmAr, FromElf - V4.1.0.894 
<b>Reference:</b> smsc_reusable_fw_requirement.doc */

/*******************************************************************************
 $File: //depot_pcs/FWEng/solutions/cec1712_GEN2/maincodeline/secureboot_app/src/plib/smbus/smb_master.cpp $

Version Control Information (Perforce)
*******************************************************************************
$Revision: #5 $
$DateTime: 2020/04/19 23:36:53 $
$Author: I19961 $
  Change Description:
    3. Updated with proper if condition
    2. Code changes to support Master Tx chaining
    1. Branched from //depot_pcs/FWEng/projects/MEC1324/maincodeline/libSmbus/source/smb/ 
        label - MEC1324_LIB_SMBUS_0400
*******************************************************************************/

extern "C"
{
    #include <common.h>
}

#include "smb_master.hpp"
using namespace SMB;

#ifndef DISABLE_SMB_MASTER

SMB_MASTER::SMB_MASTER(void) {}

/******************************************************************************/
/** SMB_MASTER::init function.
 * This function initializes the SMB_MASTER instance
 * @param None
 * @return None
*******************************************************************************/
void SMB_MASTER::init(const UINT8 instance_id)
{
    instance_ = instance_id;
    
    //trace1(0, SMB_MASTER, 0, "init: %d", instance_);

    state_ = (UINT8)MASTER_DISABLED;
    ptr_func_ = (MASTER_FUNC_PTR) NULLVOIDPTR;
    wdtCounter_ = 0x0;
    ptr_buffer_ = (UINT8 *)(NULLVOIDPTR);

}/* SMB_MASTER::init */

/******************************************************************************/
/** SMB_MASTER::enable function.
 * This function enables the master by setting state to idle
 * @return None
 * @param None
*******************************************************************************/
void SMB_MASTER::enable(void)
{

    state_ = (UINT8)(MASTER_IDLE);

}/* SMB_MASTER::enable */

/******************************************************************************/
/** SMB_MASTER::disable function.
 * This function disables the master
 * @return None
 * @param None
*******************************************************************************/
void SMB_MASTER::disable(void)
{
    //trace0(0, SMB_MASTER, 0, "disable: Enter");
    state_ = (UINT8)MASTER_DISABLED;
    complStat_ = (UINT8)ERROR_SMB_DISABLED;
    (void)inform_app();

}/* SMB_MASTER::disable */

/******************************************************************************/
/** SMB_MASTER::interrupt_task function.
 * This function handles the interrupt task processing for master
 * @return None
 * @param None
*******************************************************************************/
void SMB_MASTER::interrupt_task(void)
{
    /* Check for master completion */
    if ((UINT8)MASTER_DONE == state_)
    {
        //trace0(0, SMB_MASTER, 0, "interrupt_task: processing ...");
        /* Reset Master WDT Counter */
        wdtCounter_ = 0x0;
        appRetVal_ =  inform_app();
        state_ =(UINT8) MASTER_IDLE;
    }
}/* SMB_MASTER::interrupt_task */

/******************************************************************************/
/** SMB_MASTER::setup function.
 * This function set-up the master for any master
 * @param buffer_ptr - the pointer to the buffer containing master data
 * @param pecEnable - flag to indicate if PEC needs to be enabled
 * @param func_ptr - the master function pointer
 * @return None
*******************************************************************************/
void SMB_MASTER::setup(UINT8 *buffer_ptr, bool pecEnable, MASTER_FUNC_PTR func_ptr)
{

    //trace0(0, SMB_MASTER, 0, "setup: Enter ");

    state_ = (UINT8)MASTER_ENABLE;
    ptr_func_ = func_ptr;
    checkPEC_ = pecEnable;
    ptr_buffer_ = buffer_ptr;

}/* SMB_MASTER::setup */

/******************************************************************************/
/** SMB_MASTER::form_cmd_value function.
 * This function forms the master command value as per the smb protocol
 * @param smb_protocol - the smb protocol
 * @param writeCount - write count associated with the protocol
 * @return None
*******************************************************************************/
void SMB_MASTER::form_cmd_value( const UINT8 smb_protocol, UINT8 writeCount, const UINT8 i2c_readCount, 
                                                const bool rdChainedFlag, const bool wrChainedFlag)
{
    UINT8 Start0=0, Stop=0;
    UINT8 StartN =0, ReadM=0, readCount=0, pec_term=0, read_pec=0;

    //NOTE: ptr_buffer_ is assigned to application buffer in setup function

    /* For repeat protocols read bit should be set at end of tx buffer */
    if ( (smb_protocol & 0xC0) == (I2C_REPEAT << 6) )
    {
        /* Slave Address should be present in buffer_ptr[writeCount-1];*/
        mSET_BIT(BIT_0_MASK, ptr_buffer_[writeCount-1] );
    }

    switch (smb_protocol)
    {
        case SMB_SEND_BYTE:
     //case SMB_QUICK_COMMAND:
     //case SMB_WRITE_BYTE:
     //case SMB_WRITE_WORD:
     //case SMB_WRITE_BLOCK:
     //case SMB_I2C_WRITE:
        Start0 =1; Stop = 1;
        readCount = 0;
        if (writeCount > 1) /* Not a Quick Command */
        {
            pec_term = 1;
            mCLR_BIT(BIT_0_MASK, ptr_buffer_[0]);
        }
        break;

     case SMB_RECEIVE_BYTE:
         Start0 =1; Stop = 1;
         readCount = 1;
         mSET_BIT(BIT_0_MASK, ptr_buffer_[0]);
         read_pec = 1;
         break;

    case SMB_I2C_READ:
         Start0 =1; Stop = 1;
         readCount = i2c_readCount;
         mSET_BIT(BIT_0_MASK, ptr_buffer_[0]);               
         read_pec = 1;
         break;

    case SMB_I2C_COMBINED:
         Start0 =1; StartN =1; Stop = 1;
         readCount = i2c_readCount;
         mCLR_BIT(BIT_0_MASK, ptr_buffer_[0]);               
         read_pec = 1;
         break;

     case SMB_READ_BYTE:
         Start0 =1; StartN =1; Stop = 1;
         readCount = 1;
         mCLR_BIT(BIT_0_MASK, ptr_buffer_[0]);
         read_pec = 1;
         break;

     case SMB_READ_WORD:
     //case SMB_PROCESS_CALL:
         Start0 =1; StartN =1; Stop = 1;
         readCount = 2;
         mCLR_BIT(BIT_0_MASK, ptr_buffer_[0]);
         read_pec = 1;
         break;

     case SMB_READ_BLOCK:
     //case SMB_BLOCK_WRITE_BLOCK_READ_PROCESS_CALL:
         Start0 =1; StartN =1; ReadM =1; Stop = 1;
         readCount = i2c_readCount;
         mCLR_BIT(BIT_0_MASK, ptr_buffer_[0]);
         read_pec = 1;
         break;
     default:
        //trace0(0, SMB_MASTER, 0, "form_cmd_value: Invalid smb_protocol");
         break;
    }

    dmaEndAddrOffset_ = readCount + writeCount; 
    if (SMB_READ_BLOCK == smb_protocol)
    {
        dmaEndAddrOffset_ = SMB_MAX_MASTER_READ_BLOCK_SIZE + writeCount;
    }
    
    if (checkPEC_)
    {
        /* If PEC is enabled we need to read two extra bytes, one PEC byte from
         * slave, and another from the hardware for PEC verification */
        if (!rdChainedFlag)
        {
            dmaEndAddrOffset_ += (UINT8)(read_pec*2);
        }
    }
    else
    {
        read_pec = 0;
        pec_term = 0;
    }
    
    readChainedFlag_ = false;
    if (rdChainedFlag)
    {
        //Program the readCount a little more, so that clock is stretched when dma completes
        readCount += 5;
        readChainedFlag_ = true;
    }
    
    writeChainedFlag_ = false;
    if (wrChainedFlag)
    {
        //Program the writeCount a little more, so that clock is stretched when dma completes
        writeCount += 5;
        writeChainedFlag_ = true;
    }
    
    dmaEndAddrOffsetSave_ = dmaEndAddrOffset_;
    
    masterCommand_ = (UINT32)(((UINT32)readCount << 24) |   ((UINT32)writeCount << 16) |
                    ((UINT32)read_pec << 13) | ((UINT32)ReadM << 12) | ((UINT32) pec_term << 11) |
                    ((UINT32)Stop << 10) | ((UINT32) StartN << 9) | ((UINT32)Start0 << 8));

    readBlkFlag_ = false;
    if (SMB_READ_BLOCK == smb_protocol)
    {
        readBlkFlag_ = true;
    }
    
    //trace4(0, SMB_MASTER, 0, "form_cmd_value: ptr[0] = %02Xh mCmnd_ = %04Xh %04Xh dmaEndAddrOffset_ = %04Xh", ptr_buffer_[0], masterCommand_>>16, masterCommand_, dmaEndAddrOffset_);
    

}/* SMB_MASTER::form_cmd_value */


/******************************************************************************/
/** SMB_MASTER::xmit function.
 * This function starts the master state machine
 * @return None
 * @param None
*******************************************************************************/
void SMB_MASTER::xmit(void)
{
    UINT32 dataAHBAddress;
    UINT8 device;

    dataAHBAddress = I2CSMBx_MasterTxBufferAddrGet((SMB_INSTANCE)instance_);
    device = get_device_id(SMBUS_MASTER, instance_);

    /* Setup dma for transmitting */
    DMA_SetupTx((DMA_CHANNEL)dma_chan_, device, dataAHBAddress, (UINT32)ptr_buffer_, dmaEndAddrOffset_);
    
    if (writeChainedFlag_)
    {
        //trace0(0, SMB_MASTER, 0, "xmit: enable_interrupt()");
         ((DMA_CHANNEL)dma_chan_, device);
    }
    
    DMA_Start((DMA_CHANNEL)dma_chan_);

    /* Enable MDONE Interrupt */
	I2CSMBx_MdoneEnable((SMB_INSTANCE)instance_);

    /* Start the Master */
    I2CSMBx_StartMaster((SMB_INSTANCE)instance_, masterCommand_);

    wdtCounter_ = 0x0;

}/* SMB_MASTER::xmit */

/******************************************************************************/
/** SMB_MASTER::receive function.
 * This function is used for receiving using dma chaining
 * @return None
 * @param None
*******************************************************************************/
void SMB_MASTER::receive(UINT8 readCount)
{
    UINT32 dataAHBAddress;
    UINT8 device;   
    UINT8 dmaEndAddrOffsetAdjusted;
    
    dmaEndAddrOffset_ = readCount;
    dmaEndAddrOffsetAdjusted = dmaEndAddrOffset_;   
    
    device = get_device_id(SMBUS_MASTER, instance_);
    //trace2(0, SMB_MASTER, 0, "receive: readCount = %02Xh dmaEndAddrOffset_ = %02Xh", readCount, dmaEndAddrOffset_);
    
    if (APP_RETVAL_CHAINED_RX == appRetVal_)
    {
            readCount = readCount+5;
            
            //trace0(0, SMB_MASTER, 0, "receive: enable_interrupt()");
            DMA_EnableInterrupt((DMA_CHANNEL)dma_chan_);
    }
        
    if (APP_RETVAL_CHAINED_RX_LAST == appRetVal_)
    {
        readCount = readCount - 1;
        dmaEndAddrOffsetAdjusted += 5;
        if (checkPEC_)
        {
            // trace0(0, SMB_MASTER, 0, "receive: checkPEC_ is true");
             dmaEndAddrOffset_ += 2;
             dmaEndAddrOffsetAdjusted += 2;          
        }               
    }
    
    //trace3(0, SMB_MASTER, 0, "receive: readCount = %02Xh dmaEndAddrOffsetAdjusted = %02Xh checkPEC_ = %02Xh", readCount, dmaEndAddrOffsetAdjusted, checkPEC_);    

    /* Re-program the readCount in the master command register */
    I2CSMBx_MasterCmdRegReadCntSet((SMB_INSTANCE)instance_, readCount);  
    
  /* Setup dma for transmitting */  
    dataAHBAddress = I2CSMBx_MasterRxBufferAddrGet((SMB_INSTANCE)instance_);
    
    /* Enable MDONE Interrupt */
    I2CSMBx_MdoneEnable((SMB_INSTANCE)instance_);
    
    DMA_SetupRx((DMA_CHANNEL)dma_chan_, device, dataAHBAddress, (UINT32)ptr_buffer_, dmaEndAddrOffsetAdjusted, true);
    DMA_Start((DMA_CHANNEL)dma_chan_);

    wdtCounter_ = 0x0;

}/* SMB_MASTER::receive */

/******************************************************************************/
/** SMB_MASTER::xmit_chained function.
 * This function is used for transmiting using dma chaining
 * @return None
 * @param None
*******************************************************************************/
void SMB_MASTER::xmit_chained(UINT8 writeCount)
{
    UINT32 dataAHBAddress;
    UINT8 device;   
    UINT8 dmaEndAddrOffsetAdjusted;
    
    dmaEndAddrOffset_ = writeCount;
    dmaEndAddrOffsetAdjusted = dmaEndAddrOffset_;   
    
    device = get_device_id(SMBUS_MASTER, instance_);
    //trace2(0, SMB_MASTER, 0, "xmit_chained: writeCount = %02Xh dmaEndAddrOffset_ = %02Xh", writeCount, dmaEndAddrOffset_);
    
    if (APP_RETVAL_CHAINED_TX == appRetVal_)
    {
            writeCount += 5u;          
            trace0(0, SMB_MASTER, 0, "xmit_chained: enable_interrupt()");
            DMA_EnableInterrupt((DMA_CHANNEL)dma_chan_);
    }
        
    if (APP_RETVAL_CHAINED_TX_LAST == appRetVal_)
    {
        //writeCount = writeCount - 1;
        dmaEndAddrOffsetAdjusted += 5;                      
    }
    
    //trace3(0, SMB_MASTER, 0, "xmit_chained: writeCount = %02Xh dmaEndAddrOffsetAdjusted = %02Xh checkPEC_ = %02Xh", writeCount, dmaEndAddrOffsetAdjusted, checkPEC_);    

    /* Re-program the write count in the master command register */
    I2CSMBx_MasterCmdRegWriteCntSet((SMB_INSTANCE)instance_, writeCount);    
    
  /* Setup dma for transmitting */  
    dataAHBAddress = I2CSMBx_MasterTxBufferAddrGet((SMB_INSTANCE)instance_);
    
    /* Enable MDONE Interrupt */
    I2CSMBx_MdoneEnable((SMB_INSTANCE)instance_); 
    
    DMA_SetupTx((DMA_CHANNEL)dma_chan_, device, dataAHBAddress, (UINT32)ptr_buffer_, dmaEndAddrOffsetAdjusted);
    DMA_Start((DMA_CHANNEL)dma_chan_);

    wdtCounter_ = 0x0;

}/* SMB_MASTER::xmit_chained */

/******************************************************************************/
/** SMB_MASTER::proceed function.
 * This function handles the receive phase of the master transaction
 * @return None
 * @param None
*******************************************************************************/
void SMB_MASTER::proceed(void)
{
    UINT32 dataAHBAddress;
    UINT8 device;

    //trace1(0, SMB_MASTER, 0, "proceed: readChainedFlag_ = %02Xh", readChainedFlag_);

    dataAHBAddress = I2CSMBx_MasterRxBufferAddrGet((SMB_INSTANCE)instance_);
    device = get_device_id(SMBUS_MASTER, instance_);

    DMA_SwitchTxToRx((DMA_CHANNEL)dma_chan_, device, dataAHBAddress);
    if (readChainedFlag_)
    {
        //trace0(0, SMB_MASTER, 0, "proceed: enable_interrupt()");
        DMA_EnableInterrupt((DMA_CHANNEL)dma_chan_);
    }
    DMA_Start((DMA_CHANNEL)dma_chan_);

    /* Enable MDONE Interrupt */
    I2CSMBx_MdoneEnable((SMB_INSTANCE)instance_);

    /* Start the Master */
    I2CSMBx_MasterProceed((SMB_INSTANCE)instance_);

}/* SMB_MASTER::proceed */

/******************************************************************************/
/** SMB_MASTER::retry function.
 * This function retries the master transaction
 * @return None
 * @param None
*******************************************************************************/
void SMB_MASTER::retry(void)
{
    UINT8 WriteCount, smb_protocol;
    UINT8 readCount;

    if ((appRetVal_ == (UINT8)APP_RETVAL_HOLD_SMBUS) || (appRetVal_ == (UINT8)APP_RETVAL_RELEASE_SMBUS))
    {       
        return;
    }
    
    //trace2(0, SMB_MASTER, 0, "retry: appRetVal_ = %02Xh state_ = %02Xh", appRetVal_, state_);

    switch (appRetVal_)
    {
    case APP_RETVAL_RETRY:
        //trace0(0, SMB_MASTER, 0, "retry: Retrying Master Transaction");

        if ((UINT8)MASTER_IDLE == state_)
        {
            //trace0(0, SMB_MASTER, 0, "retry: Master Idle ");
            state_ =(UINT8) MASTER_ENABLE;
            if ((readChainedFlag_) || (writeChainedFlag_))
            {
                dmaEndAddrOffset_ = dmaEndAddrOffsetSave_;
            }
            //trace3(0, SMB_MASTER, 0, "retry: mCmnd_ = %04Xh %04Xh dmaEndAddrOffset_ = 04Xh", masterCommand_>>16, masterCommand_, dmaEndAddrOffset_);
            xmit();
        }
        else
        {
            //trace0(0, SMB_MASTER, 0, "retry: Master Not Idle");
            appRetVal_ = ptr_func_(instance_, (UINT8)ERROR_CLK_DATA_NOT_HIGH, ptr_buffer_, &appCbkNewTx_);
            ptr_func_ = (MASTER_FUNC_PTR)NULLVOIDPTR;
        }
        break;

    case APP_RETVAL_NEW_TX:

        ptr_buffer_ = appCbkNewTx_.buffer_ptr;

        if ((UINT8)MASTER_IDLE == state_)
        {
            //trace0(0, SMB_MASTER, 0, "retry: new_tx: Master Idle ");
            
            //trace3(0, SMB_MASTER, 0, "retry: new_tx: WriteCount = %02Xh pecEnable    = %02Xh smb_protocol = 02Xh", appCbkNewTx_.WriteCount, appCbkNewTx_.pecEnable, appCbkNewTx_.smb_protocol);


            state_ = (UINT8)MASTER_ENABLE;
            checkPEC_ = (bool)appCbkNewTx_.pecEnable;
            smb_protocol = appCbkNewTx_.smb_protocol;
            WriteCount = appCbkNewTx_.WriteCount;

            form_cmd_value(smb_protocol, WriteCount, ptr_buffer_[WriteCount], false, false);
            //trace1(0, SMB_MASTER, 0, "retry: new_tx: ptr_buffer_:[0] = %02Xh", ptr_buffer_[0]);
            //trace1(0, SMB_MASTER, 0, "retry: new_tx: ptr_buffer_:[1] = %02Xh", ptr_buffer_[1]);
            //trace1(0, SMB_MASTER, 0, "retry: new_tx: ptr_buffer_:[2] = %02Xh", ptr_buffer_[2]);
            //trace1(0, SMB_MASTER, 0, "retry: new_tx: ptr_buffer_:[3] = %02Xh", ptr_buffer_[3]);
            //trace1(0, SMB_MASTER, 0, "retry: new_tx: ptr_buffer_:[4] = %02Xh", ptr_buffer_[4]);
            xmit();

        }
        else
        {
            //trace0(0, SMB_MASTER, 0, "retry: new_tx: Master Not Idle");
            appRetVal_ = ptr_func_(instance_, (UINT8)ERROR_CLK_DATA_NOT_HIGH, ptr_buffer_, &appCbkNewTx_);
            ptr_func_ = (MASTER_FUNC_PTR)NULLVOIDPTR;
        }

        break;
        
    case APP_RETVAL_CHAINED_RX:
    case APP_RETVAL_CHAINED_RX_LAST:
        
        ptr_buffer_ = appCbkNewTx_.buffer_ptr;  
        readCount = ptr_buffer_[0];

        if ((UINT8)MASTER_IDLE == state_)
        {
            //trace0(0, SMB_MASTER, 0, "retry: chained: Master Idle ");         
            
            state_ = (UINT8)MASTER_ENABLE;
        
            receive(readCount);

        }
        else
        {
            //trace0(0, SMB_MASTER, 0, "retry: chained: Master Not Idle");
            appRetVal_ = ptr_func_(instance_, (UINT8)ERROR_CLK_DATA_NOT_HIGH, ptr_buffer_, &appCbkNewTx_);
            ptr_func_ = (MASTER_FUNC_PTR)NULLVOIDPTR;
        }
        
        break;
        
    case APP_RETVAL_CHAINED_TX:
    case APP_RETVAL_CHAINED_TX_LAST:
        
        ptr_buffer_ = appCbkNewTx_.buffer_ptr;  
        WriteCount = appCbkNewTx_.WriteCount;

        if ((UINT8)MASTER_IDLE == state_)
        {
            //trace0(0, SMB_MASTER, 0, "retry: tx chained: Master Idle ");          
            state_ = (UINT8)MASTER_ENABLE;      
            xmit_chained(WriteCount);
        }
        else
        {
            //trace0(0, SMB_MASTER, 0, "retry: tx chained: Master Not Idle");
            appRetVal_ = ptr_func_(instance_, (UINT8)ERROR_CLK_DATA_NOT_HIGH, ptr_buffer_, &appCbkNewTx_);
            ptr_func_ = (MASTER_FUNC_PTR)NULLVOIDPTR;
        }
        
        break;

    default:
         //trace0(0, SMB_MASTER_ERR, 0, "retry: Invalid case, shouldn't happen ");
        break;
    }

    //reset appRetVal_ value, it's job is done
    appRetVal_ = APP_RETVAL_RELEASE_SMBUS;

}/* SMB_MASTER::retry */

/******************************************************************************/
/** SMB_MASTER::handle_bus_error function.
 * This function handles bus error for the master
 * @return None
 * @param None
*******************************************************************************/
void SMB_MASTER::handle_bus_error(void)
{
    /* if any master transaction is currently in progress
     * or if it has just completed */
    if (((UINT8)MASTER_ENABLE == state_) ||
        ((UINT8)MASTER_DONE == state_))
    {
        //trace0(0, SMB_MASTER, 0, "handle_bus_error: inform application");
        /* Reset Master WDT Counter */
        wdtCounter_ = 0;
        (void)inform_app();
        appRetVal_=(UINT8)APP_RETVAL_RELEASE_SMBUS;
        state_ = (UINT8)MASTER_IDLE;
    }
    wdtCounter_ = 0x0;
}/* SMB_MASTER::handle_bus_error */

/******************************************************************************/
/** SMB_MASTER::dma_isr function.
 * This function is the interrupt service routine for the master
 * @return None
 * @param None
*******************************************************************************/
void SMB_MASTER::dma_isr(void)
{
    //UINT8 readCount;
    UINT8 device;

    //trace0(0, SMB_MASTER, 0, "SMB_MASTER::dma_isr ");
    DMA_Stop((DMA_CHANNEL)dma_chan_);
    DMA_DisableInterrupt((DMA_CHANNEL)dma_chan_);
    //I2CSMBx_MdoneDisable((SMB_INSTANCE)instance_);
    
    if (writeChainedFlag_)
    {
        complStat_ = (UINT8)SUCCESS_TX_CHAINED;
    }
    else
    {
        complStat_ = (UINT8)SUCCESS_RX_CHAINED;
    }
    
    state_ = (UINT8)MASTER_DONE;
        
}/* SMB_MASTER::dma_isr */

/******************************************************************************/
/** SMB_MASTER::isr function.
 * This function is the interrupt service routine for the master
 * @return None
 * @param True/False - True if Interrupt service task is required
*******************************************************************************/
bool SMB_MASTER::isr(void)
{
    UINT8 readCount;
        bool intrTaskReqFlag;
      bool isInterruptTaskRequired;
	  UINT8 device;

    intrTaskReqFlag = true;
    isInterruptTaskRequired = false;  
    
      DMA_Stop((DMA_CHANNEL)dma_chan_);
      DMA_DisableInterrupt((DMA_CHANNEL)dma_chan_);
    I2CSMBx_MdoneDisable((SMB_INSTANCE)instance_);

    if (I2CSMBx_IsLABSet((SMB_INSTANCE)instance_))
    {
        //trace0(0, SMB_MASTER, 0, "isr: LAB ");
        I2CSMBx_ClrLABCompletionStatus((SMB_INSTANCE)instance_);
        complStat_ = (UINT8)ERROR_LAB;
    }
    else if (I2CSMBx_IsMNAKXSet((SMB_INSTANCE)instance_))
    {
        //trace0(0, SMB_MASTER, 0, "isr: MNAKX ");
        if ( is_master_address_nack()){
            complStat_ = (UINT8)ERROR_MADDR_NAKX;
        }
        else{
            complStat_ = (UINT8)ERROR_MDATA_NAKX;
        }
        I2CSMBx_MasterCmdSet((SMB_INSTANCE)instance_, 0x0);
    }
    else if (I2CSMBx_IsMTRSet((SMB_INSTANCE)instance_))
    {
        readCount = I2CSMBx_MasterReadCountGet((SMB_INSTANCE)instance_);
        //trace1(0, SMB_MASTER, 0, "isr: readCount = 0x%02x ", readCount);
        if (readCount){
            //trace0(0, SMB_MASTER, 0, "isr: go to master proceed ");
            /* If ReadCount proceed to master receive phase */
            I2CSMBx_ClrMasterCompletionStatus((SMB_INSTANCE)instance_);
            proceed();
            intrTaskReqFlag = false;
        }
        else {
            //trace0(0, SMB_MASTER, 0, "isr: success tx ");
            complStat_ = (UINT8)SUCCESS_TX;
        }
    }
    else/* Successful Master Tx followed by Rx */
    {
        if (!is_PEC_valid()){
            complStat_ = (UINT8)ERROR_PEC;
        }
        else {
            complStat_ = (UINT8)SUCCESS_RX;
        }
    }
    if (intrTaskReqFlag)
    {
        state_ = (UINT8)MASTER_DONE;
        I2CSMBx_ClrMasterCompletionStatus((SMB_INSTANCE)instance_);

            //kSET_EVENT_INTERRUPT(smbus);
            isInterruptTaskRequired = true;         
    }
    
    //trace1(0, SMB_MASTER, 0, "isr: complStat_ = 02Xh ", complStat_);
    
    return isInterruptTaskRequired;
        
}/* SMB_MASTER::isr */

/******************************************************************************/
/** SMB_MASTER::ber_isr function.
 * This function handles bus error interrupt handling for master
 * @return None
 * @param None
*******************************************************************************/
void SMB_MASTER::ber_isr(void)
{
    /* This check is to make sure we update smbMasterCompletionStatus only
     * when it has already not completed, otherwise we end up notifying application
     * with BER even if master had completed successfully */
    if ((UINT8)MASTER_DONE != state_)
    {
        if (I2CSMBx_IsTimerErrorSet((SMB_INSTANCE)instance_))
        {
            //trace0(0, SMB_MASTER, 0, "ber_isr: Timeout");
            complStat_ = (UINT8)ERROR_BER_TIMEOUT;
        }
        else
        {
            //trace0(0, SMB_MASTER, 0, "ber_isr: Non Timeout");
            complStat_ = (UINT8)ERROR_BER_NON_TIMEOUT;
        }
    }

    /* If master was enabled, we need to inform the application, so set state to MASTER_DONE*/
    if ((UINT8)MASTER_ENABLE == state_)
    {
        state_ = (UINT8)MASTER_DONE;
    }

    I2CSMBx_ClrAllCompletionStatus((SMB_INSTANCE)instance_);
}/* SMB_MASTER::ber_isr */

/******************************************************************************/
/** SMB_MASTER::check_busy function.
 * This function checks if the master is available for any transaction
 * @return MASTER_BUSY/MASTER_AVAILABLE
 * @note This function is a overloaded function,
 * see UINT8 SMB_MASTER::check_busy(MASTER_FUNC_PTR app_func_ptr) for the other version
*******************************************************************************/
UINT8 SMB_MASTER::check_busy(void) const
{
    /* Check if Master is Ready for a transaction */
    if ((UINT8)MASTER_IDLE != state_)
    {
        //trace0(0, SMB_MASTER, 0, "check_busy: Master Busy ");
        return (UINT8)MASTER_BUSY;
    }

#if 0
    /* Check if Master is acquired by some other application */
    if ((MASTER_FUNC_PTR)NULLVOIDPTR != ptr_func_)
    {
        //trace0(0, SMB_MASTER, 0, "check_busy: Master acquired by some other application ");
        return (UINT8)MASTER_BUSY;
    }
#endif    

    return MASTER_AVAILABLE;
}/* SMB_MASTER::check_busy */

/******************************************************************************/
/** SMB_MASTER::check_busy function.
 * This function checks if the master is available for any transaction for a
 * particular application
 * @param app_func_ptr function pointer of the application function
 * @return MASTER_BUSY/MASTER_AVAILABLE
 * @note This function is a overloaded function,
 * see UINT8 SMB_MASTER::check_busy(void) for the other version
*******************************************************************************/
UINT8 SMB_MASTER::check_busy(MASTER_FUNC_PTR app_func_ptr) const
{
    /* Check if Master is Ready for a transaction */
    if ((UINT8)MASTER_IDLE != state_)
    {
        //trace0(0, SMB_MASTER, 0, "check_busy: Master Busy ");
        return (UINT8)MASTER_BUSY;
    }

#if 0
    /* Check if Master is acquired by some other application */
    if (((MASTER_FUNC_PTR)NULLVOIDPTR != ptr_func_) && (app_func_ptr != ptr_func_))
    {
        //trace0(0, SMB_MASTER, 0, "check_busy: Master acquired by some other application ");
        return (UINT8)MASTER_BUSY;
    }
#endif    

    return MASTER_AVAILABLE;
}/* SMB_MASTER::check_busy */

/******************************************************************************/
/** SMB_MASTER::is_ready_for_port_change function.
 * This function checks if the master is ready for port change operation
 * @return TRUE/FALSE
*******************************************************************************/
bool SMB_MASTER::is_ready_for_port_change(void) const
{
    bool retVal = true;
    /* Check if Master is Ready for a port change operation */
    if (!(((UINT8)MASTER_IDLE == state_) || ((UINT8)MASTER_DONE == state_)))
    {
        //trace0(0, SMB_MASTER, 0, "is_ready_for_port_change: Master Busy ");
        retVal = false;
    }
    return retVal;
}

/******************************************************************************/
/** SMB_MASTER::inform_app function.
 * This function informs the application of the transaction status
 * @param None
 * @return application return value
*******************************************************************************/
UINT8 SMB_MASTER::inform_app(void)
{
    UINT8 retVal =(UINT8)APP_RETVAL_RELEASE_SMBUS;

    if ((MASTER_FUNC_PTR)NULLVOIDPTR != ptr_func_)
    {
        retVal = ptr_func_(instance_, complStat_, ptr_buffer_, &appCbkNewTx_);
        if ((UINT8)APP_RETVAL_RELEASE_SMBUS == retVal)
        {
            //trace0(0, SMB_MASTER, 0, "SMB_MASTER::inform_app: Application Released Smbus");
            ptr_func_ = (MASTER_FUNC_PTR)NULLVOIDPTR;
        }
    }
    return retVal;
}/* SMB_MASTER::inform_app */

/******************************************************************************/
/** SMB_MASTER::is_wdt_expired function.
 * This function checks if master wdt timer has expired
 * @return TRUE/FALSE
 * @param None
*******************************************************************************/
bool SMB_MASTER::is_wdt_expired(void)
{
    bool retVal = false;

    /* If master is enabled and master wdt is enabled */
    if (((UINT8)MASTER_ENABLE == state_) && (wdtInterval_ != 0x0))
    {
        wdtCounter_++;

        //trace2(0, SMB_MASTER, 0, "is_wdt_expired: wdtCounter_ %02Xh / %02Xh ", wdtCounter_, wdtInterval_);

        if (wdtCounter_ == (wdtInterval_ + 1))
        {
            //trace0(0, SMB_MASTER, 0, "is_wdt_expired: TRUE ");
            retVal = true;
        }
    }
    return retVal;
}/* SMB_MASTER::is_wdt_expired */

/******************************************************************************/
/** SMB_MASTER::is_master_address_nack function.
 * This function checks if the NACK that is received is at address byte or
 * data byte
 * @return TRUE/FALSE
 * @param None
*******************************************************************************/
bool SMB_MASTER::is_master_address_nack(void)
{
    UINT8 writeCount;
    UINT8 writeCountPrev;
    bool retVal = false;

    writeCount = I2CSMBx_MasterWriteCountGet((SMB_INSTANCE)instance_);
    writeCountPrev = (masterCommand_ & 0x00FF0000) >> 16;
    //trace2(0, SMB_MASTER, 0, "is_master_address_nack: MasterWriteCount start = %02Xh; now = %02Xh",
    //      writeCountPrev, writeCountPrev);

    if (writeCount == (writeCountPrev-1))
    {
        ///trace0(0, SMB_MASTER, 0, "is_master_address_nack: Address NACK ");
        retVal = true;
    }
    else
    {
        //trace0(0, SMB_MASTER, 0, "is_master_address_nack: DATA NACK ");
    }
    return retVal;

}/* SMB_MASTER::is_master_address_nack */

/******************************************************************************/
/** SMB_MASTER::is_PEC_valid function.
 * This function checks if PEC is valid for the completed transaction
 * @return TRUE/FALSE
 * @param None
*******************************************************************************/
bool SMB_MASTER::is_PEC_valid(void) const
{
    UINT8 pecIndexInBuffer;
    bool retVal=true;

    if (!checkPEC_)
    {
        //Checking of PEC not required
        return retVal;
    }

    if (readBlkFlag_)
    {
        /* For read block, the number of bytes read is stored in location [3],
         * So pecIndex will be number of bytes read + number of bytes transmitted +
         * byte read at location [3] + pec byte,
         * i.e. master_buffer_ptr[3] + 3 + 1 + 1;
         */
        pecIndexInBuffer = ptr_buffer_[3] + 5;
    }
    else
    {
        pecIndexInBuffer = dmaEndAddrOffset_ - 1;
    }
    //trace1(0, SMB_MASTER, 0, "is_pec_valid: pecIndexInBuffer = %02X", pecIndexInBuffer);

    if (ptr_buffer_[pecIndexInBuffer])
    {
        //trace2(0, SMB_MASTER, 0, "is_pec_valid: PEC Invalid: SlavePEC: %02X; H/W PEC: %02X ", 
         //      ptr_buffer_[pecIndexInBuffer-1], ptr_buffer_[pecIndexInBuffer]);
        retVal = false;
    }
    return retVal;

}/* SMB_MASTER::is_pec_valid */

/** @}
*/
#endif //DISABLE_SMB_MASTER
