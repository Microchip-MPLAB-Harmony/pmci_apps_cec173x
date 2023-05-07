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
/** @defgroup smb smb_slave
 *  @{
 */
/** @file smb_slave.cpp
 \brief the smb slave source file
 This file implements the smb slave functionality. It provides the definitions of
 function declared in SMB_SLAVE class

<b>Platform:</b> This is ARM-based component
  Details of hardware blocks on which this smbus reuse component is tested:
    smbus HW Block name - blk_i2c_spb, smbus HW Block build # - B0076   
    dma HW Block name - blk_dma_ahb, dma HW Block build # - B0007
    FPGA build used for testing -> MEC1322 FPGA build Rev-81-Randy - 081_chip_top_fpga2_run1
    //depotAE/projects/MEC1322/FPGA builds/Rev-81-Randy/081_chip_top_fpga2_run1.mcs

<b>Toolset:</b> MDK-Lite V 4.23; Armcc, Armasm, ArmLink, ArmAr, FromElf - V4.1.0.894 
<b>Reference:</b> smsc_reusable_fw_requirement.doc */

/*******************************************************************************
 $File: //depot_pcs/FWEng/solutions/cec1712_GEN2/maincodeline/secureboot_app/src/plib/smbus/smb_slave.cpp $

Version Control Information (Perforce)
*******************************************************************************
$Revision: #3 $
$DateTime: 2020/01/22 07:39:45 $
$Author: snakka $
  Change Description:    
    2. Bug fix for losing active slave buffers when smbus is disabled and enabled
    1. Branched from //depot_pcs/FWEng/projects/MEC1324/maincodeline/libSmbus/source/smb/ 
       label - MEC1324_LIB_SMBUS_0400
*******************************************************************************/
extern "C"
{
    #include <common.h>
}

#include "smb_slave.hpp"

using namespace SMB;

extern bool is_bus_error_flag_set(const UINT8 channel);

SMB_SLAVE::SMB_SLAVE(void) {}

/******************************************************************************/
/** SMB_SLAVE::init function.
 * This function is called during intitialization to setup the slave state
 * @param buffer_ptr pointer to a allocated buffer for the slave to use
 * @return None
*******************************************************************************/
void SMB_SLAVE::init(UINT8 instance_id, UINT8 *buffer_ptr, UINT8 buffer_size)
{
      instance_ = instance_id;
      dma_chan_ = (instance_id * 2);
    
        //trace1(0, SMB_SLAVE, 0, "init: %d", instance_);
    
#ifndef DISABLE_ALL_SLAVE_BUFFERS     
    state_ = (UINT8)SLAVE_DISABLED;
    repeatReadFlag_ = false;
    slaveTxInProgressUsingDMA = false;
    slaveRxDMAInterrupt = false;    
    bufferMgmt_.init(buffer_ptr, buffer_size);
    nackMechanism_ = (UINT8)SLAVE_NO_BUFFER_NACK;
#else
    ack_read_count_ = 0xFF;
#endif

}/* SMB_SLAVE::init */

/******************************************************************************/
/** SMB_SLAVE::enable function.
 * This function enables the slave and dma as per buffer availability
 * @param None
 * @return None
 * @note This function is called during initialization and through slave isr
*******************************************************************************/
void SMB_SLAVE::enable(void)
{
    //trace0(0, SMB_SLAVE, 0, "enable: Enter");

#ifndef DISABLE_ALL_SLAVE_BUFFERS

    repeatReadFlag_ = false;

    /* Get buffer from Buffer Management Layer */
     active_buffer_ = bufferMgmt_.free_buffer_get();

    if (NULLVOIDPTR == active_buffer_)
    {
        if ((UINT8)SLAVE_NO_BUFFER_NACK == nackMechanism_)
        {
            //trace0(0, SMB_SLAVE, 0, "enable: Buffer Not Available, configuring for NACK");
            state_ = (UINT8)SLAVE_NACK;
            configure(&spare_buffer_[0], READCOUNT_ONE, true, true, true, false);
        }
        else //nackMechanism_ = SLAVE_LAST_BUFFER_CLK_STRETCH
        {
            //trace0(0, SMB_ERR, 0, "SMB_SLAVE::enable: Buffer Not Available, something wrong");
        }

        return;

    }
    /* If clock stretch is the mechanism and if this is the last buffer, configure
     * for clock stretch. We also check for buffers count > 1 because for
     * clock stretch mechanism we need atleast 2 buffers, since we wait for
     * pending buffer to get freed */
    if (((UINT8)SLAVE_LAST_BUFFER_CLK_STRETCH == nackMechanism_) &&
       ((! bufferMgmt_.is_there_any_free_buffer()) && (bufferMgmt_.get_buffers_count() > 1)))
    {
       // trace0(0, SMB_SLAVE, 0, "enable: Enabling clock stretching");
        state_ = (UINT8)SLAVE_CLK_STRETCH;
    }
    else /* 'Not last buffer' or 'Not Clock Stretch Mode' */
    {
       state_ = (UINT8)SLAVE_ENABLE;
    }
    active_buffer_->XmitCount = 0;
    active_buffer_->RxCount = 0;
    active_buffer_->sdoneFlag = false;
    active_buffer_->channel = instance_;
#ifdef ENABLE_SLAVE_RX_MORE_THAN_256BYTES
    configure(active_buffer_->buffer_ptr, bufferMgmt_.get_buffer_size(), false, (state_ ==(UINT8)SLAVE_ENABLE), true, true);
#else
    configure(active_buffer_->buffer_ptr, bufferMgmt_.get_buffer_size(), false, (state_ ==(UINT8)SLAVE_ENABLE), true, false);
#endif

#else
    configure(&spare_buffer_[0], ack_read_count_, true, true, false, false);
#endif

}/* SMB_SLAVE::enable */

/******************************************************************************/
/** SMB_SLAVE::enable from isr function.
 * This function enables the slave and dma as per buffer availability
 * @param None
 * @return None
 * @note This function is called during initialization and through slave isr
*******************************************************************************/
void SMB_SLAVE::enable_isr(void)
{
    //trace0(0, SMB_SLAVE, 0, "enable: Enter");

#ifndef DISABLE_ALL_SLAVE_BUFFERS

    repeatReadFlag_ = false;

    /* Get buffer from Buffer Management Layer */
     active_buffer_ = bufferMgmt_.free_buffer_get();

    if (NULLVOIDPTR == active_buffer_)
    {
        if ((UINT8)SLAVE_NO_BUFFER_NACK == nackMechanism_)
        {
            //trace0(0, SMB_SLAVE, 0, "enable: Buffer Not Available, configuring for NACK");
            state_ = (UINT8)SLAVE_NACK;
            configure_isr(&spare_buffer_[0], READCOUNT_ONE, true, true, true, false);
        }
        else //nackMechanism_ = SLAVE_LAST_BUFFER_CLK_STRETCH
        {
            //trace0(0, SMB_ERR, 0, "SMB_SLAVE::enable: Buffer Not Available, something wrong");
        }

        return;

    }
    /* If clock stretch is the mechanism and if this is the last buffer, configure
     * for clock stretch. We also check for buffers count > 1 because for
     * clock stretch mechanism we need atleast 2 buffers, since we wait for
     * pending buffer to get freed */
    if (((UINT8)SLAVE_LAST_BUFFER_CLK_STRETCH == nackMechanism_) &&
       ((! bufferMgmt_.is_there_any_free_buffer()) && (bufferMgmt_.get_buffers_count() > 1)))
    {
       // trace0(0, SMB_SLAVE, 0, "enable: Enabling clock stretching");
        state_ = (UINT8)SLAVE_CLK_STRETCH;
    }
    else /* 'Not last buffer' or 'Not Clock Stretch Mode' */
    {
       state_ = (UINT8)SLAVE_ENABLE;
    }
    active_buffer_->XmitCount = 0;
    active_buffer_->RxCount = 0;
    active_buffer_->sdoneFlag = false;
    active_buffer_->channel = instance_;
#ifdef ENABLE_SLAVE_RX_MORE_THAN_256BYTES
    configure_isr(active_buffer_->buffer_ptr, bufferMgmt_.get_buffer_size(), false, (state_ ==(UINT8)SLAVE_ENABLE), true, true);
#else
    configure_isr(active_buffer_->buffer_ptr, bufferMgmt_.get_buffer_size(), false, (state_ ==(UINT8)SLAVE_ENABLE), true, false);
#endif

#else
    configure_isr(&spare_buffer_[0], ack_read_count_, true, true, false, false);
#endif

}/* SMB_SLAVE::enable */

/******************************************************************************/
/** SMB_SLAVE::disable function.
 * This function disables the slave
 * @param None
 * @return flag to indicate if event task scheduling is required
*******************************************************************************/
#ifndef DISABLE_PENDING_PACKETS
bool SMB_SLAVE::disable(void)
{
    bool eventTaskReqFlag;
    
    eventTaskReqFlag = false;
    //trace0(0, SMB_SLAVE, 0, "disable: Enter");

#ifndef DISABLE_ALL_SLAVE_BUFFERS   
    //changed order - first free active buffer and then change slave state to disabled
    free_active_buffer(); 
    state_ = (UINT8)SLAVE_DISABLED;
    eventTaskReqFlag = check_and_process_recvd_pkts();
#else
    I2CSMBx_SdoneDisable((SMB_INSTANCE)instance_);
#endif
    
    return eventTaskReqFlag;

}/* SMB_SLAVE::disable */
#else
void SMB_SLAVE::disable(void)
{   
    //trace0(0, SMB_SLAVE, 0, "disable: Enter");

#ifndef DISABLE_ALL_SLAVE_BUFFERS    
    free_active_buffer();
    state_ = (UINT8)SLAVE_DISABLED;
    check_and_process_recvd_pkts();
#else
    I2CSMBx_SdoneDisable((SMB_INSTANCE)instance_);
#endif

}/* SMB_SLAVE::disable */
#endif


/******************************************************************************/
/** SMB_SLAVE::configure function.
* This function configures smbus controller for slave mode
* @param buffer_ptr     Buffer for receiving a packet
* @param readCount     Buffer readCount
* @param configureForNackFlag   Flag to indicate if configuring for NACK
* @param dmaEnableFlag          Flag to indicate if dma needs to be enabled
* @return None
*******************************************************************************/
void SMB_SLAVE::configure(const UINT8 *const buffer_ptr,const UINT8 readCount,const bool configureForNackFlag
                            ,bool dmaEnableFlag,const bool incrMemAddrFlag, const bool enableDmaInterrupt)
{
    UINT32 dataAHBAddress;
    UINT8 device;

    //trace0(0, SMB_SLAVE, 0, "configure: Enter");

    dataAHBAddress = I2CSMBx_SlaveRxBufferAddrGet((SMB_INSTANCE)instance_);
    device = get_device_id(SMBUS_SLAVE, instance_);

    if (configureForNackFlag)
    {
        dmaEnableFlag = true;
    }
    
    if (enableDmaInterrupt)
    {        
         /* We enable dma interrupt to support slave Rx more than 256 bytes.
         * Programming length in dma less than in smb network layer, will cause 
         * dma interrupt and clock to stretch. This provides opportunity to 
         * re-configure the readcount in smb network layer */
        DMA_SetupRx((DMA_CHANNEL)dma_chan_, device, dataAHBAddress, (UINT32)buffer_ptr, readCount -2, incrMemAddrFlag);
        DMA_EnableInterrupt((DMA_CHANNEL)dma_chan_);
    }
    else
    {

        /* Configure the dma buffer size to a little more than actual ReadCount */         
        DMA_SetupRx((DMA_CHANNEL)dma_chan_, device, dataAHBAddress, (UINT32)buffer_ptr, readCount + 5, incrMemAddrFlag);
    }

    if (dmaEnableFlag)
    {
        DMA_Start((DMA_CHANNEL)dma_chan_);
    }
    I2CSMBx_SdoneEnable((SMB_INSTANCE)instance_);
    I2CSMBx_StartSlave((SMB_INSTANCE)instance_, readCount, 0, false);
}/* SMB_SLAVE::configure */

/******************************************************************************/
/** SMB_SLAVE::configure from isr function.
* This function configures smbus controller for slave mode
* @param buffer_ptr     Buffer for receiving a packet
* @param readCount     Buffer readCount
* @param configureForNackFlag   Flag to indicate if configuring for NACK
* @param dmaEnableFlag          Flag to indicate if dma needs to be enabled
* @return None
*******************************************************************************/
void SMB_SLAVE::configure_isr(const UINT8 *const buffer_ptr,const UINT8 readCount,const bool configureForNackFlag
                            ,bool dmaEnableFlag,const bool incrMemAddrFlag, const bool enableDmaInterrupt)
{
    UINT32 dataAHBAddress;
    UINT8 device;

    //trace0(0, SMB_SLAVE, 0, "configure: Enter");

    dataAHBAddress = I2CSMBx_SlaveRxBufferAddrGet((SMB_INSTANCE)instance_);
    device = get_device_id(SMBUS_SLAVE, instance_);

    if (configureForNackFlag)
    {
        dmaEnableFlag = true;
    }
    
    if (enableDmaInterrupt)
    {        
         /* We enable dma interrupt to support slave Rx more than 256 bytes.
         * Programming length in dma less than in smb network layer, will cause 
         * dma interrupt and clock to stretch. This provides opportunity to 
         * re-configure the readcount in smb network layer */
        DMA_SetupRx((DMA_CHANNEL)dma_chan_, device, dataAHBAddress, (UINT32)buffer_ptr, readCount -2, incrMemAddrFlag);
        DMA_EnableInterrupt((DMA_CHANNEL)dma_chan_);
    }
    else
    {

        /* Configure the dma buffer size to a little more than actual ReadCount */         
        DMA_SetupRx((DMA_CHANNEL)dma_chan_, device, dataAHBAddress, (UINT32)buffer_ptr, readCount + 5, incrMemAddrFlag);
    }

    if (dmaEnableFlag)
    {
        DMA_Start((DMA_CHANNEL)dma_chan_);
    }
    I2CSMBx_SdoneEnable((SMB_INSTANCE)instance_);
    I2CSMBx_StartSlave((SMB_INSTANCE)instance_, readCount, 0, false);
}/* SMB_SLAVE::configure */

/******************************************************************************/
/** SMB_SLAVE::inform_app_and_reconfigure function.
 * This function informs application of the received data and continues the slave 
 * Rx transaction by reconfiguring
 * @param None
 * @return None
 * @note Application should provide the data immediately, it can't postpone it as in
 *       the case of other received packets (e.g Write Block). Application should provide
 *       the data from the start of the buffer. Application should also indicate the PEC
 *       flag and the data length to transmit.
*******************************************************************************/
void SMB_SLAVE::inform_app_and_reconfigure(void)
{
    UINT32 dataAHBAddress;
    UINT8 device;
    UINT8 status;
    UINT8 readCount = READCOUNT_ONE;

     //trace0(0, SMB_SLAVE, 0, "inform_app_and_reconfigure: Enter ");

     /* Get Slave data from application */        
     status = dispatch_.inform_app(active_buffer_, SLAVE_TRANSMIT_FALSE);

     dataAHBAddress = I2CSMBx_SlaveRxBufferAddrGet((SMB_INSTANCE)instance_);
     device = get_device_id(SMBUS_SLAVE, instance_);

     /* If application has accepted the slave data, status will be STATUS_BUFFER_DONE */
     if ((UINT8)STATUS_BUFFER_DONE == status)
     {
          readCount = bufferMgmt_.get_buffer_size(); 
          DMA_SetupRx((DMA_CHANNEL)dma_chan_, device, dataAHBAddress, (UINT32)(active_buffer_->buffer_ptr), readCount-2, true);
          DMA_EnableInterrupt((DMA_CHANNEL)dma_chan_);
     }
     else
     {
         /* Note: If status returned is not STATUS_BUFFER_DONE, 
      *        ReadCount will be 1 (initialized value); we will configure to receive only few more bytes and NACK further */
         DMA_SetupRx((DMA_CHANNEL)dma_chan_, device, dataAHBAddress, (UINT32)(active_buffer_->buffer_ptr), readCount+5, true);
     }
    
//    AHB_API_setup_rx((DMA_CHANNEL)dma_chan_, device, dataAHBAddress, (UINT32)(active_buffer_->buffer_ptr), readCount-1, true);
//    DMA_EnableInterrupt((DMA_CHANNEL)dma_chan_);
    
    /* Note: SDONE is already enabled */
    
    I2CSMBx_StartSlave((SMB_INSTANCE)instance_, readCount, 0, false);
        
    DMA_Start((DMA_CHANNEL)dma_chan_);
    
}/* SMB_SLAVE::inform_app_and_reconfigure */


/******************************************************************************/
/** SMB_SLAVE::proceed function.
 * This function initiates the slave transmit phase
 * @param None
 * @return None
*******************************************************************************/
void SMB_SLAVE::proceed(const UINT8 *const buffer_ptr,const UINT8 writeCount,const bool pecFlag, const bool slaveXmitDoneFlag)
{
    UINT32 dataAHBAddress;
    UINT8 device;

     trace1(0, SMB_SLAVE, 0, "proceed: slaveXmitDoneFlag = %02Xh", slaveXmitDoneFlag ); 

     dataAHBAddress = I2CSMBx_SlaveTxBufferAddrGet((SMB_INSTANCE)instance_);
     device = get_device_id(SMBUS_SLAVE, instance_);

     DMA_SetupTx((DMA_CHANNEL)dma_chan_, device, dataAHBAddress, (UINT32)buffer_ptr, writeCount);

     if (!slaveXmitDoneFlag)
     {
        DMA_EnableInterrupt((DMA_CHANNEL)dma_chan_);
         //trace0(0, SMB_SLAVE, 0, "proceed: enable_interrupt() ");
        //Program the writecount a little more, so that clock is stretched when dma completes
        I2CSMBx_StartSlave((SMB_INSTANCE)instance_, 0, writeCount+1, pecFlag);
        slaveTxInProgressUsingDMA = true;
     }
     else
     {
        I2CSMBx_StartSlave((SMB_INSTANCE)instance_, 0, writeCount, pecFlag);
        slaveTxInProgressUsingDMA = false;
     }


     DMA_Start((DMA_CHANNEL)dma_chan_);

}/* SMB_SLAVE::proceed */


/******************************************************************************/
/** SMB_SLAVE::handle_bus_error function.
 * This function handles bus error processing for slave
 * @param None
 * @return None
*******************************************************************************/
void SMB_SLAVE::handle_bus_error(void)
{
    trace0(0, SMB_SLAVE, 0, "handle_bus_error: Enter ");

#ifndef DISABLE_ALL_SLAVE_BUFFERS
    free_active_buffer();
    state_ = (UINT8)SLAVE_RESET_INITIALIZE;
    repeatReadFlag_ = false;
    slaveTxInProgressUsingDMA = false;
    slaveRxDMAInterrupt = false; 
#else
    I2CSMBx_SdoneDisable((SMB_INSTANCE)instance_);
#endif

}/* SMB_SLAVE::handle_bus_error */

#ifdef DISABLE_ALL_SLAVE_BUFFERS

/******************************************************************************/
/** SMB_SLAVE::isr function.
 * This function handles interrupt service routine for slave
 * @param None
 * @return True if detected error condition, else False
 * @note The error condition will be detected when configured to ack all bytes
 *  and we are receiving more number of bytes than configured. In this case
 *  we treat it as bus error and reset ourselves.
*******************************************************************************/
bool SMB_SLAVE::isr(void)
{
    UINT8 ReadCount;
    UINT8 volatile flag1=0;  /* Coverity fix */
    bool retVal;    
    uint8_t device;

    retVal = false;

   device = get_device_id(SMBUS_MASTER, instance_);
    /* Stop the Slave DMA */
    DMA_Stop((DMA_CHANNEL)dma_chan_);

    if (I2CSMBx_IsSTRSet((SMB_INSTANCE)instance_))
    {
        //trace0(0, SMB_SLAVE, 0, "isr: Slave Xmit Done: ");
        flag1 = 1;
        I2CSMBx_ClrSlaveCompletionStatus((SMB_INSTANCE)instance_);
        configure(&spare_buffer_[0], ack_read_count_, true, true, false, false);
    }
    else if (I2CSMBx_IsSNAKRSet((SMB_INSTANCE)instance_))
    {
        //trace0(0, SMB_SLAVE, 0, "isr: SNAKR Set ");
        flag1 = 2;
       I2CSMBx_ClrSlaveCompletionStatus((SMB_INSTANCE)instance_);
        retVal = true;
    }
    else if (I2CSMBx_IsRepeatWriteSet((SMB_INSTANCE)instance_))
    {
        //trace0(0, SMB_SLAVE, 0, "isr: Repeat Write Set: ");
        flag1 = 3;
        I2CSMBx_ClrSlaveCompletionStatus((SMB_INSTANCE)instance_);
        I2CSMBx_StartSlave((SMB_INSTANCE)instance_, 0, 0, false);
    }
    else if (I2CSMBx_IsRepeatReadSet((SMB_INSTANCE)instance_))
    {
        //trace0(0, SMB_SLAVE, 0, "isr: RepeatRead Set ");
        flag1 = 4;
        I2CSMBx_SlaveTxBufferFlush((SMB_INSTANCE)instance_);
        I2CSMBx_ClrSlaveCompletionStatus((SMB_INSTANCE)instance_);
        spare_buffer_[0] = 0xFF;
        proceed(spare_buffer_, 1, false);
    }
    else
    {
        ReadCount = I2CSMBx_SlaveReadCountGet((SMB_INSTANCE)instance_);
        //trace1(0, SMB_SLAVE, 0, "isr: packet received; ReadCount = %d", ReadCount);

        /* Check for receive byte protocol, it will be handled similar to Repeat Read */
        if ((((ack_read_count_-1) == ReadCount) && (mGET_BIT(READ_BIT, spare_buffer_[0]))))
        {
            //trace0(0, SMB_SLAVE, 0, "isr: Receive Byte ");
         I2CSMBx_ClrSlaveCompletionStatus((SMB_INSTANCE)instance_);
            spare_buffer_[0] = 0xFF;
            proceed(spare_buffer_, 1, false);
            flag1 = 5;
        }
        else
        {
            I2CSMBx_ClrSlaveCompletionStatus((SMB_INSTANCE)instance_);
            configure(&spare_buffer_[0], ack_read_count_, true, true, false, false);
            flag1 = 6;
        }
    }
    
   // TRACE0(53, SMB_SLAVE, 0, "isr: flag1 = %02Xh", flag1);

    return retVal;

}/* SMB_SLAVE::isr */

#endif

#ifndef DISABLE_ALL_SLAVE_BUFFERS
/******************************************************************************/
/** SMB_SLAVE::isr function.
 * This function handles interrupt service routine for slave
 * @param None
 * @return True/False - True if interrupt task service is required
*******************************************************************************/
bool SMB_SLAVE::isr(void)
{
    UINT8 ReadCount=0;
    UINT8 volatile flag1=0;
    bool isInterruptTaskRequired;
    UINT8 device;

//  trace1(0, SMB_SLAVE, 0, "isr: state = %02Xh", state_);
    
    isInterruptTaskRequired = false;

    /* Stop the Slave DMA */
    DMA_Stop((DMA_CHANNEL)dma_chan_);
    DMA_DisableInterrupt((DMA_CHANNEL)dma_chan_);

    if ((UINT8)SLAVE_CLK_STRETCH == state_)
    {
            //trace0(0, SMB_SLAVE_ERR, 0, "isr: SLAVE_CLK_STRETCH something wrong : ");
            I2CSMBx_ClrSlaveCompletionStatus((SMB_INSTANCE)instance_);
            //smb_slave_enable(Channel);
            flag1 =1;
    }
    else if ((UINT8)SLAVE_NACK == state_)
    {
        //trace1(0, SMB_SLAVE, 0, "isr: SLAVE_NACK done, BufferForNack = %02Xh", spare_buffer_[0]);
        if (!I2CSMBx_IsSNAKRSet((SMB_INSTANCE)instance_))
        {
            //trace0(0, SMB_SLAVE_ERR, 0, "isr: SNAKR not set:, something wrong ");
            flag1 =2;
        }
        I2CSMBx_ClrSlaveCompletionStatus((SMB_INSTANCE)instance_);
        enable_isr();
        flag1 =3;
    }
    else if (I2CSMBx_IsSTRSet((SMB_INSTANCE)instance_))
    {
            //trace0(0, SMB_SLAVE, 0, "isr: Slave Xmit Done: ");
            I2CSMBx_ClrSlaveCompletionStatus((SMB_INSTANCE)instance_);
            active_buffer_->XmitCount = 0;
            active_buffer_->RxCount = 0;
            active_buffer_->sdoneFlag = false;
        #ifdef ENABLE_SLAVE_RX_MORE_THAN_256BYTES
            configure_isr(active_buffer_->buffer_ptr, bufferMgmt_.get_buffer_size(), false, true, true, true);
        #else
            configure_isr(active_buffer_->buffer_ptr, bufferMgmt_.get_buffer_size(), false, true, true, false);
        #endif  
            //trace1(0, SMB_SLAVE, 0, "smb_slave_configure: SlaveXmitDone buffer_ptrForConfg %04Xh ", (UINT16)(UINT32)active_buffer_->buffer_ptr);
            flag1 =4;
    }
    else if ( I2CSMBx_IsSNAKRSet((SMB_INSTANCE)instance_))
    {
            //trace0(0, SMB_SLAVE, 0, "isr: SNAKR Set ");
            I2CSMBx_ClrSlaveCompletionStatus((SMB_INSTANCE)instance_);
            active_buffer_->XmitCount = 0;
            active_buffer_->RxCount = 0;
            active_buffer_->sdoneFlag = false;
        #ifdef ENABLE_SLAVE_RX_MORE_THAN_256BYTES
            configure_isr(active_buffer_->buffer_ptr, bufferMgmt_.get_buffer_size(), false, true, true, true);
        #else
            configure_isr(active_buffer_->buffer_ptr, bufferMgmt_.get_buffer_size(), false, true, true, false);
        #endif            
            //trace1(0, SMB_SLAVE, 0, "smb_slave_configure: SNAKR, buffer_ptrForConfg %04Xh ", (UINT16)(UINT32)active_buffer_->buffer_ptr);
            flag1 =5;
    }
    else if (I2CSMBx_IsRepeatWriteSet((SMB_INSTANCE)instance_))
    {
            //trace0(0, SMB_SLAVE, 0, "isr: Repeat Write Set: ");
            active_buffer_->XmitCount = 0;
            active_buffer_->RxCount = 0;
            active_buffer_->sdoneFlag = false;
            I2CSMBx_ClrSlaveCompletionStatus((SMB_INSTANCE)instance_);
            I2CSMBx_StartSlave((SMB_INSTANCE)instance_, 0, 0, false);
            flag1 =6;
    }
    else if (I2CSMBx_IsRepeatReadSet((SMB_INSTANCE)instance_))
    {
            ReadCount = I2CSMBx_SlaveReadCountGet((SMB_INSTANCE)instance_);
            //trace2(0, SMB_SLAVE, 0, "isr: RepeatRead Set; ReadCount = %d Datalen = %d", ReadCount, active_buffer_->DataLen);
            active_buffer_->DataLen = bufferMgmt_.get_buffer_size() - ReadCount;
            active_buffer_->RxCount++;
            active_buffer_->sdoneFlag = true;
            //trace1(0, SMB_SLAVE, 0, "isr: Datalen = %d", active_buffer_->DataLen);
            I2CSMBx_SlaveTxBufferFlush((SMB_INSTANCE)instance_);
            repeatReadFlag_ = true;
            I2CSMBx_ClrSlaveCompletionStatus((SMB_INSTANCE)instance_);
            /* Schedule event task for executing smb_slave_transmit function */
            //kSET_EVENT_INTERRUPT(smbus);
            isInterruptTaskRequired = true;
            flag1 =7;
    }
    else
    {
        ReadCount = I2CSMBx_SlaveReadCountGet((SMB_INSTANCE)instance_);
            //trace1(0, SMB_SLAVE, 0, "isr: packet received; ReadCount = %d", ReadCount);
            //trace2(0, SMB_SLAVE, 0, "isr: RepeatRead Set; ReadCount = %d Datalen = %d", ReadCount, active_buffer_->DataLen);
            active_buffer_->DataLen = bufferMgmt_.get_buffer_size() - ReadCount;
            //trace1(0, SMB_SLAVE, 0, "isr: Datalen = %d", active_buffer_->DataLen);

        active_buffer_->RxCount++;
        active_buffer_->sdoneFlag = true;

        /* Check for receive byte protocol, it will be handled similar to Repeat Read */
        if (((1 == active_buffer_->DataLen) &&
                         (mGET_BIT(READ_BIT, active_buffer_->buffer_ptr[0]))))
        {
            //trace0(0, SMB_SLAVE, 0, "isr: Receive Byte ");
            repeatReadFlag_ = true;
            I2CSMBx_ClrSlaveCompletionStatus((SMB_INSTANCE)instance_);
        /* Schedule event task for executing smb_slave_transmit function */
            //kSET_EVENT_INTERRUPT(smbus);
            isInterruptTaskRequired = true;
            flag1 =8;
        }
        else
        {
            update_pec();
#if SMB_FREERTOS
            active_buffer_->TimeStamp = (UINT16)smb_get_current_timestamp();
#elif SMB_SKERN
            active_buffer_->TimeStamp = (UINT16)(kGET_TICKS());
#else
#endif            
            bufferMgmt_.recvd_buffer_put(active_buffer_);
            I2CSMBx_ClrSlaveCompletionStatus((SMB_INSTANCE)instance_);
            enable_isr();
      /* Schedule interrupt task for executing smb_packet_received function */
            //kSET_EVENT_INTERRUPT( smbus );
            isInterruptTaskRequired = true;
            flag1 =9;
         }
    }
    
    //trace2(0, SMB_SLAVE, 0, "isr: state = %02Xh flag1 = %02Xh ", state_, flag1);
    //trace3(0, SMB_SLAVE, 0, "isr: ReadCount = %d Datalen = %d buffer_ptrForConfg %04Xh ", ReadCount, active_buffer_->DataLen, (UINT16)(UINT32)active_buffer_->buffer_ptr );
    
    flag1 += 1;

    return isInterruptTaskRequired;

}/* SMB_SLAVE::isr */


/******************************************************************************/
/** SMB_SLAVE::dma_isr function.
 * This function handles interrupt service routine for slave called from dma isr
 * @param None
 * @return None
*******************************************************************************/
void SMB_SLAVE::dma_isr(void)
{
    UINT8 ReadCount=0;
    UINT8 device;
    //trace0(0, SMB_SLAVE, 0, "dma_isr: Enter ");

    /* Stop the Slave DMA */
   DMA_Stop((DMA_CHANNEL)dma_chan_);
     DMA_DisableInterrupt((DMA_CHANNEL)dma_chan_);

    //We use the dma interrupt when the slave needs to transmit
    //more number of bytes using chained approach
     if (slaveTxInProgressUsingDMA)
     {
        repeatReadFlag_ = true;
     }
     else /* Else case is when dma interrupt happens during Slave RX phase */
     {
         ReadCount = I2CSMBx_SlaveReadCountGet((SMB_INSTANCE)instance_);
         active_buffer_->DataLen = bufferMgmt_.get_buffer_size() - ReadCount;
#if SMB_FREERTOS
         active_buffer_->TimeStamp = (UINT16)smb_get_current_timestamp();
#elif SMB_SKERN
         active_buffer_->TimeStamp = (UINT16)(kGET_TICKS());
#else 
#endif
         active_buffer_->RxCount++;
         slaveRxDMAInterrupt = true;
     }
    /* Schedule event task for executing smb_slave_transmit function */
    //kSET_EVENT_INTERRUPT(smbus);

}/* SMB_SLAVE::dma_isr */

/******************************************************************************/
/** SMB_SLAVE::interrupt_task function.
 * This function handles interrupt task processing for slave
 * @param None
 * @return flag to indicate if event task request is required for further processing
*******************************************************************************/
#ifndef DISABLE_PENDING_PACKETS
bool SMB_SLAVE::interrupt_task(void)
#else
void SMB_SLAVE::interrupt_task(void)
#endif
{

#ifndef DISABLE_PENDING_PACKETS 
     bool eventTaskReqFlag; 
    eventTaskReqFlag = false;
#endif
    
#ifndef DISABLE_PENDING_PACKETS 
     eventTaskReqFlag = check_and_process_recvd_pkts();
#else
   check_and_process_recvd_pkts();
#endif  

    /* Check if Slave Transmit or Slave Rx phase needs to be executed */
    if ( repeatReadFlag_ || slaveRxDMAInterrupt)
    {
       //trace0(0, SMB_SLAVE, 0, "interrupt_task: RepeatRead ");

       /* Check again for received packets before processing repeat read packets*/
       if ( bufferMgmt_.is_there_any_received_packet())
       {
            /* If still more packets received, process repeat read protocol packets
             * in next interrupt task schedule, so that received packets are processed
             * before repeat read protocol packet */
            //trace0(0, SMB_SLAVE, 0, "interrupt_task: More received packets ");
        }
        else if (repeatReadFlag_)
        {
            /* Proceed to slave transmit */
                    repeatReadFlag_ = false;
            get_data_and_transmit();

        }
        else if (slaveRxDMAInterrupt) /* Dma interrupt for chaining Rx */
        {
            /* Proceed to chained slave Rx */
            slaveRxDMAInterrupt = false;
            inform_app_and_reconfigure();            
        }
    }
#ifndef DISABLE_PENDING_PACKETS             
        return eventTaskReqFlag;
#endif      
}/* SMB_SLAVE::interrupt_task */

#ifndef DISABLE_PENDING_PACKETS
/******************************************************************************/
/** SMB_SLAVE::event_task function.
 * This function handles slave event task processing
 * @param None
 * @return None
*******************************************************************************/
bool SMB_SLAVE::event_task(void)
{
    //flag to indicate if further event task processing is required
    bool evenTaskFlag;
    
    //trace0(0, SMB_SLAVE, 0, "event_task: Enter ");    
    evenTaskFlag = false;
    
    evenTaskFlag = check_and_process_pending_pkts();
    
    return evenTaskFlag;

}/* SMB_SLAVE::event_task */
#endif

/******************************************************************************/
/** SMB_SLAVE::get_data_and_transmit function.
 * This function initiates the slave transmit phase by getting data from application
 * @param None
 * @return None
 * @note Application should provide the data immediately, it can't postpone it as in
 *       the case of other received packets (e.g Write Block). Application should provide
 *       the data from the start of the buffer. Application should also indicate the PEC
 *       flag and the data length to transmit.
*******************************************************************************/
void SMB_SLAVE::get_data_and_transmit(void)
{
    UINT8 status;
    UINT8 WriteCount=0;
    bool  pecFlag = false;

     //trace0(0, SMB_SLAVE, 0, "get_data_and_transmit: Enter ");

     /* Get Slave data from application */
     active_buffer_->slaveXmitDoneFlag = 1;
     status = dispatch_.inform_app(active_buffer_, SLAVE_TRANSMIT_TRUE);
     active_buffer_->XmitCount++;

     /* If application has provided slave data, status will be STATUS_BUFFER_DONE */
     if ((UINT8)STATUS_BUFFER_DONE == status)
     {
          pecFlag = (bool)active_buffer_->pecFlag;
          WriteCount = active_buffer_->DataLen;
          if (WriteCount > bufferMgmt_.get_buffer_size())
          {
              trace1(0, SMB_SLAVE, 0, "transmit: Invalid WriteCount %02Xh ",WriteCount );
              /* WriteCount needs to be set to zero for invalid WriteCount */
              WriteCount = 0x0;
          }
     }
     /* Note: If status returned is not STATUS_BUFFER_DONE, pecFlag will be FALSE
      *       and WriteCount will be 0 (initialized values) */

     proceed(active_buffer_->buffer_ptr, WriteCount, pecFlag, active_buffer_->slaveXmitDoneFlag);

}/* SMB_SLAVE::transmit */


/******************************************************************************/
/** SMB_SLAVE::free_active_buffer function.
 * This function frees the slave buffer that is currently configured
 * @param None
 * @return None
*******************************************************************************/
void SMB_SLAVE::free_active_buffer(void)
{
    if (((UINT8)SLAVE_ENABLE == state_) || ((UINT8)SLAVE_CLK_STRETCH == state_))
    {
        I2CSMBx_SdoneDisable((SMB_INSTANCE)instance_);
        bufferMgmt_.free_buffer_put(active_buffer_);
    }

}/* SMB_SLAVE::free_active_buffer */

/******************************************************************************/
/** SMB_SLAVE::check_and_process_recvd_pkts function.
 * This function checks if any received packets and then calls function to
 * process them
 * @param None
 * @return flag to indicate if event task request is required
*******************************************************************************/
#ifndef DISABLE_PENDING_PACKETS
bool SMB_SLAVE::check_and_process_recvd_pkts(void)
{
    bool eventTaskReqFlag;
    
    eventTaskReqFlag = false;
     /* Check for received packets */
    if ( bufferMgmt_.is_there_any_received_packet())
    {
       //trace0(0, SMB_SLAVE, 0, "check_and_process_recvd_pkts: packet present");
       eventTaskReqFlag = process_received_packets();       
    }
    return eventTaskReqFlag;
}/* SMB_SLAVE::check_and_process_recvd_pkts */

#else
void SMB_SLAVE::check_and_process_recvd_pkts(void)
{
     /* Check for received packets */
    if ( bufferMgmt_.is_there_any_received_packet())
    {
       //trace0(0, SMB_SLAVE, 0, "check_and_process_recvd_pkts: packet present");
       process_received_packets();      
    }
}/* SMB_SLAVE::check_and_process_recvd_pkts */
#endif

/******************************************************************************/
/** SMB_SLAVE::check_and_process_pending_pkts function.
 * This function checks if any pending packets and then calls function to
 * process them
 * @param None
 * @return flag indicating if further event task processing is required
*******************************************************************************/
#ifndef DISABLE_PENDING_PACKETS
bool SMB_SLAVE::check_and_process_pending_pkts(void)
{
    bool eventTaskFlag;
    
    eventTaskFlag = false;
    
    /* Check for pending packets */
    if ( bufferMgmt_.is_there_any_pending_packet())
    {
        //trace0(0, SMB_SLAVE, 0, "check_and_process_pending_pkts: packet present");
        eventTaskFlag = process_pending_packets();
    }
    
    return eventTaskFlag;
}/* SMB_SLAVE::check_and_process_pending_pkts */
#endif

/******************************************************************************/
/** SMB_SLAVE::process_received_packets function.
 * This function calls application function for each packet received.
 * If application is busy, puts packet into Pending Buffer List
 * @param None
 * @return flag to indicate if event task processing is required
*******************************************************************************/
#ifndef DISABLE_PENDING_PACKETS
bool SMB_SLAVE::process_received_packets(void)
#else
void SMB_SLAVE::process_received_packets(void)
#endif
{
    BUFFER_INFO * received_slave_buffer_info;   
    UINT8 numPacketsProcessed=0;
#ifndef DISABLE_PENDING_PACKETS 
    bool eventTaskReqFlag;
    eventTaskReqFlag = false;
#endif
    uint8_t device;
    
    //trace0(0, SMB_SLAVE, 0, "process_received_packets: Enter ");

    device = get_device_id(SMBUS_MASTER, instance_);
  /* Process the received packets from Received Buffer List */
    I2CSMBx_SdoneDisable((SMB_INSTANCE)instance_);
    received_slave_buffer_info = bufferMgmt_.received_buffer_get();
    I2CSMBx_SdoneEnable((SMB_INSTANCE)instance_);

    while (NULLVOIDPTR!=received_slave_buffer_info)
    {
         dispatch_.inform_app(received_slave_buffer_info,SLAVE_TRANSMIT_FALSE);

 #ifndef DISABLE_PENDING_PACKETS        
        if ((UINT8)STATUS_BUFFER_NOT_DONE == status)
        {
            /* If application is busy, put the packet in Pending List */
            bufferMgmt_.pending_buffer_put(received_slave_buffer_info);
            //trace0(0, SMB_SLAVE, 0, "process_received_packets: Buffer not Done ");
        }
        else /* STATUS_BUFFER_DONE,STATUS_BUFFER_ERROR */
 #endif         
        {
            /* If application has copied the packet or no application identified
             * put the packet in Free List */
            
            /* Note if the pending packets are not enabled, packet not picked up 
             * application (i.e. STATUS_BUFFER_NOT_DONE) are discarded, that is the buffer
             * is put back to the free list */
            
            I2CSMBx_SdoneDisable((SMB_INSTANCE)instance_);
            bufferMgmt_.free_buffer_put(received_slave_buffer_info);
            //trace0(0, SMB_SLAVE, 0, "smb_slave_pkt_recved: Buffer Done ");
            if (((UINT8)SLAVE_CLK_STRETCH == state_)
                    && (!is_bus_error_flag_set(instance_)))
            {
                //trace0(0, SMB_SLAVE, 0, "process_received_packets: starting slave dma ");
                state_ = (UINT8)SLAVE_ENABLE;
                DMA_Start((DMA_CHANNEL)dma_chan_);
            }
            I2CSMBx_SdoneEnable((SMB_INSTANCE)instance_);
        }

        numPacketsProcessed++;

        if (SMB_MAX_NUM_SLAVE_BUFFER == numPacketsProcessed)
        {
            //trace0(0, SMB_SLAVE, 0, "process_received_packets: Maximum packets processed for this round");
            break;
        }

        /* Get next received packet */
        I2CSMBx_SdoneDisable((SMB_INSTANCE)instance_);
        received_slave_buffer_info = bufferMgmt_.received_buffer_get();
        I2CSMBx_SdoneEnable((SMB_INSTANCE)instance_);
    }

    #ifndef DISABLE_PENDING_PACKETS
    /* If any pending packets, schedule task event to execute
     * smb_slave_pendingPkts_hndlr function */
    if ( bufferMgmt_.is_there_any_pending_packet())
    {
          //trace0(0, SMB_SLAVE, 0, "process_received_packets: More pending packets ");
          //kSET_EVENT_TASK(smbus);
            eventTaskReqFlag = true;
    }

    return eventTaskReqFlag;
    #endif  

}/* SMB_SLAVE::process_received_packets */

#ifndef DISABLE_PENDING_PACKETS
/******************************************************************************/
/** SMB_SLAVE::process_pending_packets function.
 * This function calls application function for each packet in Pending Buffer
 * List. If application is still busy puts packet back into Pending Buffer List
 * @param None
 * @return flag to indicate if further event task processing is required
*******************************************************************************/
bool SMB_SLAVE::process_pending_packets(void)
{
    UINT8 i, count, status;
    BUFFER_INFO * pending_slave_buffer_info;
    bool eventTaskFlag;

    eventTaskFlag = false;
    //trace0(0, SMB_SLAVE, 0, "process_pending_packets: Enter ");

    count = bufferMgmt_.get_count_pendingBufferList();

  /* Process the pending packets from Pending Buffer List */
    for (i=0;i<count;i++)
    {
        pending_slave_buffer_info = bufferMgmt_.pending_buffer_get();

        if (NULLVOIDPTR != pending_slave_buffer_info)
        {
                status = dispatch_.inform_app(pending_slave_buffer_info,SLAVE_TRANSMIT_FALSE);

                if ((UINT8)STATUS_BUFFER_NOT_DONE == status)
                {
                    /* Application is still busy */
                    if ( bufferMgmt_.is_timestamp_expired(pending_slave_buffer_info->TimeStamp, SMB_PACKET_EXPIRE_TIME_TICKS))
                    {
                        //trace0(0, SMB_SLAVE, 0, "process_pending_packets: Pending packet timestamp expired ");
                        /* If packet timestamp has expired, put the buffer in Free List */
                        I2CSMBx_SdoneDisable((SMB_INSTANCE)instance_);
                        bufferMgmt_.free_buffer_put(pending_slave_buffer_info);
                        I2CSMBx_SdoneEnable((SMB_INSTANCE)instance_);
                    }
                    else
                    {
                        //trace0(0, SMB_SLAVE, 0, "process_pending_packets: Pending packet not done ");
                        /* put the packet back into Pending List */
                        bufferMgmt_.pending_buffer_put(pending_slave_buffer_info);
                        eventTaskFlag = true;
                    }
                }
                else /* STATUS_BUFFER_DONE,STATUS_BUFFER_ERROR */
                {
                    //trace0(0, SMB_SLAVE, 0, "process_pending_packets: Pending packet done ");
                    /* If application has copied the packet or no application identified
                     * put the packet in Free List */
                    I2CSMBx_SdoneDisable((SMB_INSTANCE)instance_);
                    bufferMgmt_.free_buffer_put(pending_slave_buffer_info);
                    I2CSMBx_SdoneEnable((SMB_INSTANCE)instance_);
                }       
      }
        else //if (NULLVOIDPTR != pending_slave_buffer_info)
        {           
            //trace0(0, SMB_SLAVE_ERR, 0, "process_pending_packets: No Pending Packets, shouldn't happen.. ");
            //trace0(0, SMB_SLAVE_ERR, 0, "... since we have already checked count");                       
        }
    }
    
    return eventTaskFlag;

}/* SMB_SLAVE::process_pending_packets */
#endif

/******************************************************************************/
/** SMB_SLAVE::update_pec function.
 * This function updates pecFlag based on hardware PEC value
 * @param None
 * @return None
*******************************************************************************/
void SMB_SLAVE::update_pec(void)
{
    UINT8 pecIndexInBuffer;

    pecIndexInBuffer = active_buffer_->DataLen;
    //trace1(0, SMB_SLAVE, 0, "update_pec: pecIndexInBuffer = %d", pecIndexInBuffer);
    active_buffer_->pecFlag = false;
    if (0x0 == active_buffer_->buffer_ptr[pecIndexInBuffer])
    {
        active_buffer_->pecFlag = true;
    }
}/* SMB_SLAVE::update_pec */
#endif

/** @}
*/


