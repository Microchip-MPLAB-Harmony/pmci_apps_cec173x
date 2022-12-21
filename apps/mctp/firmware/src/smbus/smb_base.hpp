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
/** @defgroup smb smb_base
 *  @{
 */
/** @file smb_base
 \brief the smb base hpp file
  This file implements the header for smb base functionality. It defines the
  SMB_BASE class

<b>Platform:</b> This is ARM-based component
  Details of hardware blocks on which this smbus reuse component is tested:
    smbus HW Block name - blk_i2c_spb, smbus HW Block build # - B0076   
    dma HW Block name - blk_dma_ahb, dma HW Block build # - B0007
    FPGA build used for testing -> MEC1322 FPGA build Rev-81-Randy - 081_chip_top_fpga2_run1
    //depotAE/projects/MEC1322/FPGA builds/Rev-81-Randy/081_chip_top_fpga2_run1.mcs

<b>Toolset:</b> MDK-Lite V 4.23; Armcc, Armasm, ArmLink, ArmAr, FromElf - V4.1.0.894 
<b>Reference:</b> smsc_reusable_fw_requirement.doc */

/*******************************************************************************
 $File: //depot_pcs/FWEng/solutions/cec1712_GEN2/maincodeline/secureboot_app/src/plib/smbus/smb_base.hpp $

Version Control Information (Perforce)
*******************************************************************************
$Revision: #3 $
$DateTime: 2020/03/16 23:39:48 $
$Author: I19961 $
  Change Description:
    3. Branched from //depotAE/ReUse_Repository/FW_ARC/driver/smbus/work_bench/source/
    4,5. Updated for smbus 1Mhz support
    6. a. Simplified smbus speed configuration
       b. Updated file header for MEC1322 test environment  
*******************************************************************************/

#ifndef SMB_BASE_HPP_
#define SMB_BASE_HPP_

#include "smb_config.h"
#include "smb_hw.hpp"
#include "smb_port.hpp"
#include "smb.h"
#include "smb_master.hpp"

namespace SMB{

/* configuration bit definitions */
enum config_bits
{
    sbit_CONFG_ENABLE                   = BIT_0_MASK    
    ,sbit_UNUSED                        = BIT_1_MASK
    //earlier was sbit_CONFG_SPEED_SELECT
    //but now redundant due to speed_cfg_ variable
     
    ,sbit_CONFG_FAIRNESS_ENABLE         = BIT_2_MASK
    ,sbit_CONFG_TIMEOUTS_ENABLE         = BIT_5_MASK
    ,sbit_CONFG_DISABLE_FORCE_TIMEOUT   = BIT_6_MASK
    ,sbit_ENABLE_CLK_LOW_AT_POWER_ON    = BIT_7_MASK
};

class SMB_BASE{

    //private:
    public:

//        HW_SMB& hw_smb_;
#ifndef DISABLE_SMB_MASTER
//        DMA::HW_DMA& hw_master_dma_;
#endif
//        DMA::HW_DMA& hw_slave_dma_;
        SMB_PORT port_[SMB_MAX_PORT_PER_CHANNEL];
        UINT16 ownAddress_;
      UINT8 instance_;
        UINT8 config_;
        UINT8 speed_cfg_;
        UINT8 portConfig_;
#ifdef HANDLE_POWER_LOW_TO_HIGH_EVENT   
    UINT8 powerLowToHighDetectFlag_;
#endif  
        UINT8 berCounter_;
        UINT8 timeoutsFlag_;
        bool busErrorFlag_;
        bool disableFlag_;

        SMB_BASE(const SMB_BASE& r); //no copy
        SMB_BASE& operator=(const SMB_BASE& r);  //no assignment

    //public:
#ifndef DISABLE_SMB_MASTER
        SMB_BASE(UINT32 hw_smb_address, UINT32 hw_master_dma_address, UINT32 hw_slave_dma_address);
        SMB_BASE(void);
#else
        SMB_BASE(UINT32 hw_smb_address, UINT32 hw_slave_dma_address);
#endif

        void init(UINT8 instance_id);
        void enable(void);
        void disable(void);
        void enable_port(void);
        bool timer_task(void);

        void ber_isr(void);
        void handle_bus_error(void);
#ifndef DISABLE_SMB_MASTER
        void handle_master_wdt(void);
#endif
        void handle_bus_error_postprocessing_ports(void);
#ifdef HANDLE_POWER_LOW_TO_HIGH_EVENT           
        bool handle_power_low_to_high_event(void);
#endif      
        bool is_need_to_disable(void);
        UINT8 get_port_state(UINT8 idx_port) const;
        void set_speed(UINT8 speed) const;
    
        /******************************************************************************/
        /** change_port function.
         * This function changes the port in the controller
         * @return None
         * @param None
        *******************************************************************************/
        void change_port( const UINT8 port) const
        {
			I2CSMBx_PortSet((SMB_INSTANCE)instance_, port);

        }/* change_port */


        /******************************************************************************/
        /** is_dyanamic_port_switching_enabled function.
         * This function checks if dynamic port switching is enabled on the controller
         * @return TRUE/FALSE
         * @param None
         ******************************************************************************/
        bool is_dyanamic_port_switching_enabled(void) const
        {
            return static_cast<bool>(mGET_BIT(sbit_PORT_CONFG_DYNAMIC_SWTCH, portConfig_));

        }/* is_dyanamic_port_switching_enabled */

        /******************************************************************************/
        /** is_bus_error function.
         * This function checks if bus error flag is set
         * @return TRUE/FALSE
         * @param None
         ******************************************************************************/
        bool is_bus_error(void) const
        {
            return  busErrorFlag_;
        }/* is_bus_error */

        /******************************************************************************/
        /** is_fairnessEnable_set function.
         * This function gets the fairnessEnable bit which indicates whether fairnessEnable
         * is enabled or not
         * @return TRUE if fairnessEnable is set else FALSE
         * @param None
         ******************************************************************************/
        bool is_fairnessEnable_set(void) const
        {
            return static_cast<bool>(mGET_BIT(sbit_CONFG_FAIRNESS_ENABLE, config_));

        }/* get_fairnessEnable  */

        /******************************************************************************/
        /** is_clk_low_at_power_on_enabled function.
         * This function checks if clk low on power ON is enabled or not
         * @return TRUE if enabled else FALSE
         * @param None
         ******************************************************************************/
        bool is_clk_low_at_power_on_enabled(void) const
        {
            return static_cast<bool>(mGET_BIT(sbit_ENABLE_CLK_LOW_AT_POWER_ON, config_));

        }/* is_clk_low_at_power_on_enabled  */
        
        /******************************************************************************/
        /** is_timeouts_enabled function.
         * This function checks if Timeout is enabled by default 
         * @return TRUE if enabled else FALSE
         * @param None
         ******************************************************************************/
        bool is_timeouts_enabled() const
        {
            return static_cast<bool>(mGET_BIT(sbit_CONFG_TIMEOUTS_ENABLE, config_));

        }/* is_clk_low_at_power_on_enabled  */


        /******************************************************************************/
        /** does_bus_error_needs_processing function.
         * This function checks if bus error bit is set in the hardware
         * @return TRUE/FALSE
         * @param None
         ******************************************************************************/
        bool does_bus_error_needs_processing(void) const
        {
            return I2CSMBx_IsBusErrorSet((SMB_INSTANCE)instance_);
        }/* does_bus_error_needs_processing */

        /******************************************************************************/
        /** does_mdone_needs_processing function.
         * This function checks if master needs processing based on MDONE
         * @return TRUE/FALSE
         * @param None
         ******************************************************************************/
        bool does_mdone_needs_processing(void) const
        {
            return I2CSMBx_IsMdoneSet((SMB_INSTANCE)instance_);
        }/* does_mdone_needs_processing */

        /******************************************************************************/
        /** does_sdone_needs_processing function.
         * This function checks if slave needs processing based on MDONE
         * @return TRUE/FALSE
         * @param None
         ******************************************************************************/
        bool does_sdone_needs_processing(void) const
        {
            return I2CSMBx_IsSdoneSet((SMB_INSTANCE)instance_);
        }/* does_sdone_needs_processing */

        /******************************************************************************/
        /** get_default_port function.
         * This function gets the default port
         * @return default port
         * @param None
         ******************************************************************************/
        UINT8 get_default_port(void) const
        {
            return (portConfig_ & DEFAULT_PORT_MASK);
        }/* get_default_port */
        
        /******************************************************************************/
        /** set_default_port function.
         * This function sets the default port
         * @return default port
         * @param None
         ******************************************************************************/
        void set_default_port(UINT8 port) 
        {
            port = port & DEFAULT_PORT_MASK;
            portConfig_ = portConfig_ | port;
        }/* set_default_port */


        /******************************************************************************/
        /** get_own_address.
         * This function gets the smb own address
         * @return ownAddress
         * @param None
         ******************************************************************************/
        UINT16 get_own_address(void) const
        {
            return ownAddress_;
        }/* get_own_address  */

        /******************************************************************************/
        /** get_disable_flag function.
         * This function gets the disable flag value
         * @return TRUE/FALSE
         * @param None
         ******************************************************************************/
        bool get_disable_flag(void) const
        {
            return disableFlag_;
        }/* get_disable_flag */

        /******************************************************************************/
        /** get_speed function.
         * This function gets speed configuration
         * @return speed code
         * @param None
         ******************************************************************************/
        UINT8 get_speed(void) const
        {           
                return  speed_cfg_;

        }/* get_speed */
        /******************************************************************************/
        /** update_speed_config_value function
         * This function updates base configuration value
         * @return None
         * @param value - the configuration value
         ******************************************************************************/
        void update_speed_config_value(const UINT8 configValue)
        {
            speed_cfg_ = configValue;
        }/* update_speed_config_value */

        /******************************************************************************/
        /** get_ber_counter_value
         * This function returns bus error counter value
         * @return bus error counter value
         * @param None
         ******************************************************************************/
        UINT8 get_ber_counter_value(void) const
        {
            return berCounter_;
        }/* update_own_addresss  */

        /******************************************************************************/
        /** update_ber_counter_value
         * This function update bus error counter value
         * @return None
         * @param value
         ******************************************************************************/
        void update_ber_counter_value(const UINT8 value)
        {
            berCounter_ = value;
        }/* update_ber_counter_value  */

        /******************************************************************************/
        /** set_disable_flag function.
         * This function sets the disable flag value
         * @return None
         * @param value - TRUE or FALSE
         ******************************************************************************/
        void set_disable_flag(const bool value)
        {
            disableFlag_ = value;
        }/* set_disable_flag */
#ifdef HANDLE_POWER_LOW_TO_HIGH_EVENT
        /******************************************************************************/
        /** set_power_low_to_high_event function.
         * This function sets the powerLowToHighDetectFlag_
         * @return None
         * @param value - TRUE or FALSE
         ******************************************************************************/
        void set_power_low_to_high_event(const bool value)
        {
            powerLowToHighDetectFlag_ = value;
        }/* set_disable_flag */
#endif        

        /******************************************************************************/
        /** set_clk_value function.
         * This function sets the bus clock register
         * @return None
         * @param value - clock value
         ******************************************************************************/
        void set_clk_value(const UINT16 clk_value)
        {
            I2CSMBx_ClkValueSet((SMB_INSTANCE)instance_, clk_value);
        }

        /******************************************************************************/
        /** set_timeoutScaling_value function.
         * This function sets the timeout scaling register
         * @return None
         * @param value - value
         ******************************************************************************/
        void set_timeoutScaling_value(const UINT32 clk_value)
        {
            I2CSMBx_TimeoutScalingValueSet((SMB_INSTANCE)instance_, clk_value);
        }

        /******************************************************************************/
        /** enable_timeouts function.
         * This function enables timeouts
         * @return None
         * @param None
         ******************************************************************************/
        void enable_timeouts(const UINT8 timeoutsFlag)
        {           
            config_ = config_ | sbit_CONFG_TIMEOUTS_ENABLE;
            timeoutsFlag_ = timeoutsFlag;                 
            I2CSMBx_TimeoutsEnable((SMB_INSTANCE)instance_, timeoutsFlag);
        }

        /******************************************************************************/
        /** update_port_config_value function.
         * This function updates port configuration information
         * @return None
         * @param value - the port configuration value
         ******************************************************************************/
        void update_port_config_value(const UINT8 value)
        {
            portConfig_ =  value;
        }/* update_port_config_value */

        /******************************************************************************/
        /** update_config_value function
         * This function updates base configuration value
         * @return None
         * @param value - the configuration value
         ******************************************************************************/
        void update_config_value(const UINT8 value)
        {
            config_ = value;
        }/* update_config_value */

        /******************************************************************************/
        /** update_own_address.
         * This function updates smb own address
         * @return None
         * @param value - the own address to update
         ******************************************************************************/
        void update_own_address(const UINT16 value)
        {
            ownAddress_ = value;
            I2CSMBx_OwnSlaveAddrSet((SMB_INSTANCE)instance_, value);
        }/* update_own_addresss  */


    } ;

}//namespace SMB

#endif /* SMB_BASE_HPP_ */
/** @}
*/
