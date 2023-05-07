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
/** @defgroup smb hw_smb
 *  @{
 */
/** @file smb_hw.cpp
 \brief the smb hw cpp file
 This file implements the hw smb functionality. This file implements the functions
 of HW_SMB class.

<b>Platform:</b> This is ARM-based component
 Details of the hardware blocks on which this smbus reuse component is tested:
    smbus HW Block name - blk_i2c_spb, smbus HW Block build # - B0076   
    dma HW Block name - blk_dma_ahb, dma HW Block build # - B0007
    FPGA build used for testing -> MEC1322 FPGA build Rev-81-Randy - 081_chip_top_fpga2_run1
    //depotAE/projects/MEC1322/FPGA builds/Rev-81-Randy/081_chip_top_fpga2_run1.mcs

<b>Toolset:</b> MDK-Lite V 4.23; Armcc, Armasm, ArmLink, ArmAr, FromElf - V4.1.0.894 
<b>Reference:</b> smsc_reusable_fw_requirement.doc */

/*******************************************************************************
 $File: //depot_pcs/FWEng/solutions/cec1712_GEN2/maincodeline/secureboot_app/src/plib/smbus/smb_hw.cpp $

Version Control Information (Perforce)
*******************************************************************************
$Revision: #1 $
$DateTime: 2019/07/31 22:19:26 $
$Author: vinayagp $
  Change Description:
    1. Branched from //depotAE/ReUse_Repository/FW_ARM/driver/smbus/work_bench/source/  
    2. Updated NOP as per platform.h
*******************************************************************************/
extern "C"
{
    #include "common.h"
}
#include "smb_hw.hpp"
#include "smb.h"

using namespace SMB;

/******************************************************************************/
/** HW_SMB::reset function.
 * This function resets the smb controller
 * @return None
 * @param None
*******************************************************************************/
void HW_SMB::reset(void)
{
    mSET_BIT(sbit_CONFIGURATION_RESET,REG32_CONFIGURATION);
    __NOP();__NOP();
    __NOP();__NOP();
    mCLR_BIT(sbit_CONFIGURATION_RESET,REG32_CONFIGURATION);
}/* HW_SMB::reset */

/******************************************************************************/
/** HW_SMB::enable function.
 * This function enables the smbus hardware controller
 * @param ownAddress - smb address to program in own address register
 * @param port - the port to configure
 * @param speed - smbus speed to configure - SMBUS_SPEED_400KHZ/SMBUS_SPEED_100KHZ
 * @param fairnessEnableFlag - flag to indicate if fairness needs to be enabled
 * @return None
*******************************************************************************/
void HW_SMB::enable(const UINT16 ownAddress, const UINT8 port,
        const UINT8 speed, const bool fairnessEnableFlag)
{
    reset();

    /* PIN =1 */
    REG8_CONTROL_STATUS = 0x80;

    /* Enable Smbus Controller and assign the port */
    REG32_CONFIGURATION = (sbit_CONFIGURATION_DISABLE_GENERAL_CALL |
                           sbit_CONFIGURATION_ENABLE | sbit_CONFIGURATION_PECEN |
                           sbit_CONFIGURATION_DFEN | port);

    /* Configure speed and other timing parameters */
    timing_init(speed, fairnessEnableFlag);

    /* Program the own address register */
    set_own_slave_address(ownAddress);

    /* PIN(Bit7 of Control Register) =1, ESO(Bit6 of Control Register)=1,
     * ACK(Bit0 of Control Register) =1 */
    REG8_CONTROL_STATUS = 0xC1;

}/* HW_SMB::enable */

/******************************************************************************/
/** HW_SMB::timing_init function.
 * This function programs the bus clock, data timing, fairness and
 * timeout scaling registers
 * @param speed - speed to configure SMBUS_SPEED_1MHZ/SMBUS_SPEED_400KHZ/SMBUS_SPEED_100KHZ
 * @param fairnessEnableFlag - flag to indicate if fairness needs to be enabled
 * @return None
*******************************************************************************/
void HW_SMB::timing_init(const UINT8 speed, const bool fairnessEnable)
{
        if (SMBUS_SPEED_400KHZ == speed)
        {
            /* For 8MHz baud clock, set the high & low period to 10,
             * therefore 8M/(10+10) = 400khz */
            //trace0(0, SMB_HW, 0, "smb_timing_init: Speed = 400KHZ");

            REG16_BUS_CLOCK = SPEED_400KHZ_BUS_CLOCK;
            REG32_DATA_TIMING = SPEED_400KHZ_DATA_TIMING;
            REG32_TIMEOUT_SCALING = SPEED_400KHZ_TIMEOUT_SCALING;
            REG8_DATA_TIMING2 = SPEED_400KHZ_DATA_TIMING_2;
            if (fairnessEnable)
            {
                //trace0(0, SMB_HW, 0, "smb_timing_init: Enabling fairness");
                mSET_BIT(sbit_CONFIGURATION_FAIR, REG32_CONFIGURATION);
                REG32_IDLE_SCALING = SPEED_400KHZ_IDLE_SCALING;
            }
        }
        else
        {
            if (SMBUS_SPEED_100KHZ == speed)
            {
                /* For 8MHz baud clock, set the high & low period to 40,
                 * therefore 8M/(40+40) = 100khz */
                //trace0(0, SMB_HW, 0, "smb_timing_init: Speed = 100KHZ");
                REG16_BUS_CLOCK = SPEED_100KHZ_BUS_CLOCK;
                REG32_DATA_TIMING = SPEED_100KHZ_DATA_TIMING;
                REG32_TIMEOUT_SCALING = SPEED_100KHZ_TIMEOUT_SCALING;
                REG8_DATA_TIMING2 = SPEED_100KHZ_DATA_TIMING_2;
                if (fairnessEnable)
                {
                    //trace0(0, SMB_HW, 0, "smb_timing_init: Enabling fairness -");
                    mSET_BIT(sbit_CONFIGURATION_FAIR, REG32_CONFIGURATION);
                    REG32_IDLE_SCALING = SPEED_100KHZ_IDLE_SCALING;
                }
            }
#if (SMB_BAUD_CLK == SMB_BAUD_CLK_16MHZ) //1Mhz i2c speed is defined only for 16Mhz baud clk
            else if (SMBUS_SPEED_1MHZ == speed)
            {
                //trace0(0, SMB_HW, 0, "smb_timing_init: Speed = 1MHZ");

                REG16_BUS_CLOCK = SPEED_1MHZ_BUS_CLOCK;
                REG32_DATA_TIMING = SPEED_1MHZ_DATA_TIMING;
                REG32_TIMEOUT_SCALING = SPEED_1MHZ_TIMEOUT_SCALING;
                REG8_DATA_TIMING2 = SPEED_1MHZ_DATA_TIMING_2;
                if (fairnessEnable)
                {
                    //trace0(0, SMB_HW, 0, "smb_timing_init: Enabling fairness");
                    mSET_BIT(sbit_CONFIGURATION_FAIR, REG32_CONFIGURATION);
                    REG32_IDLE_SCALING = SPEED_1MHZ_IDLE_SCALING;
                }
            }
#endif
            else
            {
                //trace0(0, SMB_HW, 0, "smb_timing_init: Invalid speed configuration");
            }
        }

}/* HW_SMB::timing_init */


/******************************************************************************/
/** HW_SMB::start_slave function.
 * This function starts the slave state machine by programming the slave command
 * register
 * @param read_count - slave read count
 * @param write_count - slave write count
 * @param pecFlag - flag to indicate if PEC needs to be enabled
 * @return None
*******************************************************************************/
void HW_SMB::start_slave(const UINT8 read_count, const UINT8 write_count, const bool pecFlag)
{
    UINT32 slaveCommandRegValue;
    UINT8 pec_bit=0;

    if (pecFlag)
    {
        pec_bit = 1;
    }

    slaveCommandRegValue = (UINT32)(((UINT32)read_count << 16) | ((UINT32)write_count << 8) 
                            | ((UINT32)pec_bit << 2));

    REG32_SLAVE_CMD = slaveCommandRegValue;

    /* Need to set SProceed bit in SMB Slave Commnad reg. */
    mSET_BIT(sbit_SLAVE_COMMAND_SPROCEED, REG32_SLAVE_CMD);

    /* Run the Slave State Machine */
    mSET_BIT(sbit_SLAVE_COMMAND_SRUN, REG32_SLAVE_CMD);

}/* HW_SMB::start_slave */

/******************************************************************************/
/** HW_SMB::flush_hw_tx_rx_buffers function.
 * This function clears the master/slave tx and rx buffers
 * @return None
 * @param None
*******************************************************************************/
void HW_SMB::flush_hw_tx_rx_buffers(void)
{
    /* Flush Master Xmit and Rx Buffers */
    flush_master_tx_buffer();
    flush_master_rx_buffer();

    /* Flush Slave Xmit and Rx Buffers */
    flush_slave_tx_buffer();
    flush_slave_rx_buffer();

}/* HW_SMB::flush_hw_tx_rx_buffers */

/** @}
*/
