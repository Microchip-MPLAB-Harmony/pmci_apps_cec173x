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
/** @file smb_port.hpp
 \brief the smb port header file
 This file implements the functionality for smb ports.
 It defines the SMB_PORT class

<b>Platform:</b> This is ARM-based component
  Details of hardware blocks on which this smbus reuse component is tested:
    smbus HW Block name - blk_i2c_spb, smbus HW Block build # - B0076   
    dma HW Block name - blk_dma_ahb, dma HW Block build # - B0007
    FPGA build used for testing -> MEC1322 FPGA build Rev-81-Randy - 081_chip_top_fpga2_run1
    //depotAE/projects/MEC1322/FPGA builds/Rev-81-Randy/081_chip_top_fpga2_run1.mcs

<b>Toolset:</b> MDK-Lite V 4.23; Armcc, Armasm, ArmLink, ArmAr, FromElf - V4.1.0.894 
<b>Reference:</b> smsc_reusable_fw_requirement.doc */

/*******************************************************************************
 $File: //depot_pcs/FWEng/solutions/cec1712_GEN2/maincodeline/secureboot_app/src/plib/smbus/smb_port.hpp $

Version Control Information (Perforce)
*******************************************************************************
$Revision: #1 $
$DateTime: 2019/07/31 22:19:26 $
$Author: vinayagp $
  Change Description:
    3. Branched from //depotAE/ReUse_Repository/FW_ARC/driver/smbus/work_bench/source/  
    4. Updated file header for MEC1322 test environment
*******************************************************************************/
#ifndef SMB_PORT_HPP_
#define SMB_PORT_HPP_

#include "smb.h"

namespace SMB{

#define SMB_GPIO_OUTPUT     0x300

enum clk_low_or_high{
    CLK_LOW = 0
    ,CLK_HIGH
};

/** Port states */
enum smbPortStates
{
    PORT_IDLE =0                     /**< Idle state - default/POR state */
    ,PORT_CLK_LOW                    /**< CLK line low */
    ,PORT_WAIT                       /**< Wait for longest packet duration */
    ,PORT_CHECK_CLKDATA              /**< Check for clk and data high 2nd */
    ,PORT_NOT_PRESENT                /**< Port is not present */
};

/** Port Ids */
enum smbPortIds
{
    PORT_ID_C1P1 =0              /**< Port ID Controller 1 Port 1 */
    ,PORT_ID_C1P2                    /**< Port ID Controller 1 Port 2 */
};

class SMB_PORT{

    //private:
    public:
        UINT32 clkGpioValue_; /**< GPIO control register programmed by host */
        UINT8 state_;         /**< Port State */
        UINT8 msTimerCount_;  /**< Timer count used to keep track of 10ms count */
        UINT8 id_;            /**< Port Id */
        bool berFlag_;       /**< Flag to indicate BER on the port */

        void set_clock_low(void);
        void set_clock_high(void) const;

        /******************************************************************************/
        /** get_my_channel_and_port function.
         * This function retrieves the port's channel and port number based on its id
         * @param *pChannel pointer where channel value will be updated
         * @param *pPort pointer where port value will be updated
         * @return None
        *******************************************************************************/
        void get_my_channel_and_port(UINT8 &Channel, UINT8 &Port) const
        {
            Channel = (UINT8)((id_ >> 4) & 0xF);
            Port = (UINT8)(id_ & 0xF);
        }/* get_my_channel_and_port */

    //public:

        //SMB_PORT(){};

        void init(const UINT8 channel, const UINT8 port);
        void enable(const bool force_timeout_flag);
        void state_reset(void);
        bool is_clkData_high(void) const;
        void run_state_machine(void);
        void set_clock(const UINT8 value);

        /******************************************************************************/
        /** set_bus_error_flag function.
         * This function sets the bus error flag value
         * @param None
         * @return None
        *******************************************************************************/
        void set_bus_error_flag(const bool flag)
        {
            berFlag_ = flag;
        }/* set_bus_error_flag */

        /******************************************************************************/
        /** get_state function.
         * This function retrives the port state
         * @param None
         * @return port state
        *******************************************************************************/
        UINT8 get_state(void) const
        {
            return state_;
        } /* get_state */

        /******************************************************************************/
        /** get_bus_error_flag function.
         * This function retrieves the bus error flag value
         * @param None
         * @return None
        *******************************************************************************/
        bool get_bus_error_flag(void) const
        {
            return berFlag_;
        }/* get_bus_error_flag */

    };
}

#endif /* SMB_PORT_HPP_ */
/** @}
*/
