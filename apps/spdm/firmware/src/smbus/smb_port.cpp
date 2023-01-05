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
/** @defgroup smb smb_port
 *  @{
 */
/** @file smb_port.cpp
 \brief the smb port source file
 This file implements the functionality for smb ports. It defines the functions
 declared in SMB_PORT class

<b>Platform:</b> This is ARM-based component
  Details of hardware blocks on which this smbus reuse component is tested:
    smbus HW Block name - blk_i2c_spb, smbus HW Block build # - B0076   
    dma HW Block name - blk_dma_ahb, dma HW Block build # - B0007
    FPGA build used for testing -> MEC1322 FPGA build Rev-81-Randy - 081_chip_top_fpga2_run1
    //depotAE/projects/MEC1322/FPGA builds/Rev-81-Randy/081_chip_top_fpga2_run1.mcs

<b>Toolset:</b> MDK-Lite V 4.23; Armcc, Armasm, ArmLink, ArmAr, FromElf - V4.1.0.894 
<b>Reference:</b> smsc_reusable_fw_requirement.doc */

/*******************************************************************************
 $File: //depot_pcs/FWEng/solutions/cec1712_GEN2/maincodeline/secureboot_app/src/plib/smbus/smb_port.cpp $

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

#define GET_SMB_GPIO_LOCATION
#include "smb_config.h"
#undef GET_SMB_GPIO_LOCATION

#include "smb_port.hpp"

extern SMB_CALLBACK_FUNC_PTR pSmbCallback;

using namespace SMB;

/* This function is used to calculate the number of bits set, This is needed for computing precision in 
.cpp files to fix CERT-C INT34 violations */
size_t popcount(uintmax_t num) 
{
    size_t precision = 0;
    for(;num != 0;num >>= 1)
    {
        if (num % 2 == 1)
        {
            precision++;
        }
    }
    return precision;
}

/******************************************************************************/
/** SMB_PORT::init function.
 * This function initializes the port state
 * @param None
 * @return None
*******************************************************************************/
void SMB_PORT::init(const UINT8 channel, const UINT8 port)
{
    id_ = (UINT8)(((UINT32)channel << 4) | port);

    if (0x0 == smb_gpio[channel][port].clk_gpio_control_addr)
    {
        //trace1(0, SMB_PORT, 0, "Port %u Not Defined", port);
        state_ = (UINT8)PORT_NOT_PRESENT;
        return;
    }
    state_reset();

}/* SMB_PORT::init */

/******************************************************************************/
/** SMB_PORT::enable function.
 * This function enables the port. On enabling the port holds the clk low if
 * force_timeout_flag is TRUE
 * @param force_timeout_flag - flag to indicate if force timeout needs to be done
 * @return None
*******************************************************************************/
void SMB_PORT::enable(const bool force_timeout_flag)
{
    if ((UINT8)PORT_NOT_PRESENT != state_)
    {
        if (force_timeout_flag)
        {
            //trace0(0, SMB_PORT, 0, "Enable PORT_CLK_LOW ");
            state_ = PORT_CLK_LOW;
            // Hold CLK low for 40-50 ms time interval
            set_clock(CLK_LOW);
            msTimerCount_ = HOLD_CLK_LOW_TIMER_COUNT;
        }
        else
        {
            state_ = (UINT8)PORT_WAIT;
            msTimerCount_ = WAIT_AFTER_BER_TIMER_COUNT;
        }
    }
}/* SMB_PORT::enable */

/******************************************************************************/
/** SMB_PORT::state_reset function.
 * This function resets the port variables
 * @param None
 * @return None
*******************************************************************************/
void SMB_PORT::state_reset(void)
{
    if ((UINT8)PORT_NOT_PRESENT != state_)
    {
        state_ = (UINT8)PORT_IDLE;
        berFlag_ = false;
        clkGpioValue_ = 0x0;
        msTimerCount_ =0x0;
    }
}/* SMB_PORT::state_reset */

/******************************************************************************/
/** SMB_PORT::run_state_machine function.
 * This function executes the port state machine based on current state
 * @param None
 * @return None
*******************************************************************************/
void SMB_PORT::run_state_machine(void)
{
    UINT32 count;
    
    //trace1(0, SMB_PORT, 0, "run_state_machine: port state %02Xh ",state_);
    
    switch(state_)
    {
    case PORT_CLK_LOW:
        //trace0(0, SMB_PORT, 0, " PORT_CLK_LOW ");
        msTimerCount_--;
        if (0 == msTimerCount_)
        {
            set_clock(CLK_HIGH);
            /* Wait for 40-50 ms time interval */
            msTimerCount_ = WAIT_AFTER_BER_TIMER_COUNT;
            state_ = PORT_WAIT;
        }
        break;
    case PORT_WAIT:
        //trace0(0, SMB_PORT, 0, " PORT_WAIT ");
        msTimerCount_--;
        if (0 == msTimerCount_)
        {
            state_ = PORT_CHECK_CLKDATA;
            msTimerCount_ = PERIOD_CHECK_CLK_DATA_HIGH;
        }
        break;
    case PORT_CHECK_CLKDATA:
        //trace0(0, SMB_PORT, 0, " PORT_CHECK_CLKDATA ");
        if ( 0!= msTimerCount_)
        {
            msTimerCount_--;
            if (0 == msTimerCount_)
            {
                UINT8 channel, port;
                //trace0(0, SMB_PORT, 0, "Report SMB Error to host");
                get_my_channel_and_port(channel, port);
                if (pSmbCallback)
                {
                    pSmbCallback(channel, SMB_CBK_PORT_ERROR_SET ,port);
                }
            }
        }
        
        count=0x0;
        while (count <= DETECT_CLK_DATA_HIGH_RETRY_COUNT)
        {
            if ( is_clkData_high())
            {
                UINT8 channel, port;
                trace0(0, SMB_PORT, 0, "Changing state to PORT_IDLE");
                state_ =  (UINT8)PORT_IDLE;
                get_my_channel_and_port(channel, port);
                if (pSmbCallback)
                {
                    pSmbCallback(channel, SMB_CBK_PORT_ERROR_CLR ,port);                    
                }
                break;
            }
            count++;
        }
        break;
    case PORT_IDLE:
        trace0(0, SMB_PORT, 0, " smb_reset_postProcess: Port Good ");
        break;
    default:
        //trace0(0, SMB_PORT, 0, "smb_reset_postProcess: Invalid Case");
        break;
    }
}/* SMB_PORT::run_state_machine */

/******************************************************************************/
/** SMB_PORT::is_clkData_high function.
 * This function checks if CLK and DATA lines are high
 * @param None
 * @return TRUE/FALSE
*******************************************************************************/
bool SMB_PORT::is_clkData_high(void) const
{
    UINT32 clkBit=0;
    UINT32 dataBit=0;
    UINT32 clkBitMask=0;
    UINT32 dataBitMask=0;
    UINT32 gpioInputAddrClk=0;
    UINT32 gpioInputAddrData=0;
    bool retVal = false;
    UINT8 channel;
    UINT8 port;

    //trace0(0, SMB_PORT, 0, "is_clkData_high : ");

    get_my_channel_and_port(channel, port);

    gpioInputAddrClk = smb_gpio[channel][port].input_gpio_addr_clk;
    gpioInputAddrData = smb_gpio[channel][port].input_gpio_addr_data;

    if (smb_gpio[channel][port].input_gpio_clk_bit < PRECISION(0xFFFFFFFFFFFFFFFFU)) // Coverity INT34-C
    {
        clkBitMask = (1UL << (smb_gpio[channel][port].input_gpio_clk_bit));
    }

    if (smb_gpio[channel][port].input_gpio_data_bit < PRECISION(0xFFFFFFFFFFFFFFFFU)) // Coverity INT34-C
    {
        dataBitMask = (1UL << (smb_gpio[channel][port].input_gpio_data_bit));
    }

    DISABLE_INTERRUPTS();
    clkBit = mGET_BIT(clkBitMask , (*(VUINT32 *)(gpioInputAddrClk)));
    dataBit = mGET_BIT(dataBitMask , (*(VUINT32 *)(gpioInputAddrData)));
    ENABLE_INTERRUPTS();

    if (clkBit)
    {
        //trace0(0, SMB_PORT, 0, "clkBit set ");
    }
    if (dataBit)
    {
        //trace0(0, SMB_PORT, 0, "dataBit set ");
    }

    if ( clkBit && dataBit)
    {
        //trace0(0, SMB_PORT, 0, "CLK & DATA high");
        retVal = true;
    }

    return retVal;
}/* SMB_PORT::is_clkData_high */

/******************************************************************************/
/** SMB_PORT::set_clock function.
 * This function sets the CLK line low or high based on input parameter
 * @param clk_low_or_high flag to indicate if CLK needs to be set LOW or HIGH
 * @return None
*******************************************************************************/
void SMB_PORT::set_clock(const UINT8 value)
{
    //trace0(0, SMB_PORT, 0, "set_clock ");

    if (CLK_LOW == value)
    {
        set_clock_low();
    }
    else
    {
        set_clock_high();
    }
}/* SMB_PORT::set_clock */

/******************************************************************************/
/** SMB_PORT::set_clock_low function.
 * This function sets the CLK line low
 * @param None
 * @return None
*******************************************************************************/
void SMB_PORT::set_clock_low(void)
{
    UINT32 clkGpioControlAddr;
    UINT32 clkGpioControlValue;
    UINT8 channel;
    UINT8 port;

    //trace0(0, SMB_PORT, 0, "set_clock_low ");

    get_my_channel_and_port(channel, port);

    clkGpioControlAddr = smb_gpio[channel][port].clk_gpio_control_addr;
    clkGpioControlValue = (*(VUINT32 *)(clkGpioControlAddr));

    if (SMB_GPIO_OUTPUT != clkGpioControlValue)
    {
        clkGpioValue_ = clkGpioControlValue;
    }
    (*(VUINT32 *)(clkGpioControlAddr)) = SMB_GPIO_OUTPUT;
}/* SMB_PORT::set_clock_low */

/******************************************************************************/
/** SMB_PORT::set_clock_high function.
 * This function sets the CLK line high
 * @param None
 * @return None
*******************************************************************************/
void SMB_PORT::set_clock_high(void) const
{
    UINT32 clkGpioControlAddr;
    UINT8 channel;
    UINT8 port;

    //trace0(0, SMB_PORT, 0, "set_clock_high ");

    get_my_channel_and_port(channel, port);

    clkGpioControlAddr = smb_gpio[channel][port].clk_gpio_control_addr;

    (*(VUINT32 *)(clkGpioControlAddr)) = clkGpioValue_;
}/* SMB_PORT::set_clock_high */

/** @}
*/


