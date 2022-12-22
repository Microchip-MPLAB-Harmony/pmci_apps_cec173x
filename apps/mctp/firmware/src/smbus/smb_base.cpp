/*****************************************************************************
* © 2013 Microchip Technology Inc. and its subsidiaries.
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
/** @file smb_base.cpp
 \brief the smb base cpp file
 This file implements the smb base functionality. It implements the functions for
 smb base class.

<b>Platform:</b> This is ARM-based component
  Details of hardware blocks on which this smbus reuse component is tested:
    smbus HW Block name - blk_i2c_spb, smbus HW Block build # - B0076   
    dma HW Block name - blk_dma_ahb, dma HW Block build # - B0007
    FPGA build used for testing -> MEC1322 FPGA build Rev-81-Randy - 081_chip_top_fpga2_run1
    //depotAE/projects/MEC1322/FPGA builds/Rev-81-Randy/081_chip_top_fpga2_run1.mcs

<b>Toolset:</b> MDK-Lite V 4.23; Armcc, Armasm, ArmLink, ArmAr, FromElf - V4.1.0.894 
<b>Reference:</b> smsc_reusable_fw_requirement.doc */

/*******************************************************************************
 $File: //depot_pcs/FWEng/solutions/Glacier_GEN3/maincodeline/Harmony/secureboot_app_mpu/firmware/src/smbus/smb_base.cpp $

Version Control Information (Perforce)
*******************************************************************************
$Revision: #5 $
$DateTime: 2022/09/21 04:11:31 $
$Author: i64652 $
  Change Description: Incorporated CCB comments
    3. Branched from //depotAE/ReUse_Repository/FW_ARC/driver/smbus/work_bench/source/
    4. a. Updated file header for MEC1322 test environment   
       b. Added default values for speed_cfg_ and portConfig_ in init function
*******************************************************************************/
extern "C"
{
    #include <common.h>
}

#include "smb_base.hpp"
using namespace SMB;

extern UINT8 get_master_state(const UINT8 channel);

#ifndef DISABLE_SMB_MASTER
SMB_BASE::SMB_BASE(void) {}

#else
SMB_BASE::SMB_BASE(UINT32 hw_smb_address, UINT32 hw_slave_dma_address):
                hw_smb_(* reinterpret_cast< HW_SMB *> (hw_smb_address)),
                hw_slave_dma_(* reinterpret_cast< HW_DMA *> (hw_slave_dma_address)) {}
#endif


/******************************************************************************/
/** SMB_BASE::init function.
 * This function initializes the smbus base
 * @return None
 * @param None
 ******************************************************************************/
void SMB_BASE::init(UINT8 instance_id)
{
    UINT8 idx_port; 

    instance_ = instance_id;
    
    //trace1(0, SMB_BASE, 0, "init: %d", instance_);
    
    busErrorFlag_ = false;
    disableFlag_ = false;

    speed_cfg_ = SMBUS_SPEED_100KHZ;
    portConfig_ = SMB_PORT_0;   
    //Enable dynamic port switching
    portConfig_ = portConfig_ | sbit_PORT_CONFG_DYNAMIC_SWTCH;

    for (idx_port=0; idx_port<SMB_MAX_PORT_PER_CHANNEL; idx_port++)
    {
        port_[idx_port].init(instance_, idx_port);
    }
}/* SMB_BASE::init */

/******************************************************************************/
/** SMB_BASE::enable function.
 * This function enables smb base which resets the hardware and enables it
 * @return None
 * @param None
 ******************************************************************************/
void SMB_BASE::enable(void)
{
    //trace0(0, SMB_BASE, 0, "enable: enter ");

    busErrorFlag_ = false;
//  powerLowToHighDetectFlag_ = FALSE;

    I2CSMBx_Reset((SMB_INSTANCE)instance_);

    const UINT16 ownAddress = get_own_address();
    const UINT8 defaultPort = get_default_port();
    const UINT8 speed = get_speed();
    const bool fairnessEnableFlag = is_fairnessEnable_set();

    I2CSMBx_Enable((SMB_INSTANCE)instance_, ownAddress, defaultPort, speed, fairnessEnableFlag);

}/* SMB_BASE::enable */

/******************************************************************************/
/** SMB_BASE::set_speed function.
 * This function allows to change the speed
 * @return None
 * @param None
 ******************************************************************************/
void SMB_BASE::set_speed(UINT8 speed) const
{
    //trace0(0, SMB_BASE, 0, "set_speed: enter ");
    
    const bool fairnessEnableFlag = is_fairnessEnable_set();
    
    /* Configure speed and other timing parameters */
    I2CSMBx_TimingInit((SMB_INSTANCE)instance_, speed, fairnessEnableFlag);

}/* SMB_BASE::enable */

/******************************************************************************/
/** SMB_BASE::disable function.
 * This function disables smbus base. It stops the master, slave dma and resets
 * the controller
 * @return None
 * @param None
 ******************************************************************************/
void SMB_BASE::disable(void)
{
    UINT8 idx_port;
    uint8_t device;

    //trace0(0, SMB_BASE, 0, "disable: enter ");

    /* Stop the DMA's */
#ifndef DISABLE_SMB_MASTER
    DMA_Stop((DMA_CHANNEL)((instance_*2)+1));
#endif
    DMA_Stop((DMA_CHANNEL)(instance_*2));

    /* Reset smbus controller */
    I2CSMBx_Reset((SMB_INSTANCE)instance_);

    for (idx_port=0; idx_port<SMB_MAX_PORT_PER_CHANNEL; idx_port++)
    {
        port_[idx_port].state_reset();
    }

}/* SMB_BASE::disable */

/******************************************************************************/
/** SMB_BASE::enable_port function.
 * This function enables the port on the controller. If dynamic port switching is
 * enabled, then it enables all the ports on the controller
 * @return None
 * @param None
 ******************************************************************************/
void SMB_BASE::enable_port(void)
{
    UINT8 idx_port;

    //trace0(0, SMB_BASE, 0, "enable_port: enter ");

    if (is_dyanamic_port_switching_enabled())
    {
        /* If dynamic port switching is enabled on the controller
         * all the associated ports needs to be enabled */
        for (idx_port = SMB_PORT_0; idx_port < SMB_MAX_PORT_PER_CHANNEL; idx_port++)
        {
            port_[idx_port].enable(false);
        }
    }
    else
    {
        /* Enable the default port */
        idx_port = get_default_port();
        port_[idx_port].enable(false);
    }

    /* schedule timer task for executing port state machine */
    //kSET_EVENT_WAKETIMER(ms_to_ticks(SMBUS_TIMER_TICK_COUNT), smbus);

}/* SMB_BASE::enable_port */

/******************************************************************************/
/** SMB_BASE::timer_task function.
 * This function handles timer task functionality for base. This function
 * is needed to execute the port state machine on bus errors
 * @return TRUE if further timer event scheduling is required, else FALSE
 * @param None
 ******************************************************************************/
bool SMB_BASE::timer_task(void)
{
    UINT8 idx_port;
    bool timerEventRequired = false;

    for (idx_port=0; idx_port<SMB_MAX_PORT_PER_CHANNEL; idx_port++)
    {
        if ((UINT8)PORT_NOT_PRESENT == port_[idx_port].get_state())
        {
            continue;
        }
        if ((UINT8)PORT_IDLE != port_[idx_port].get_state())
        {
           // trace2(0, SMB_BASE, 0, "timer_task: run_state_machine: instance %d port %d", instance_, idx_port);
            port_[idx_port].run_state_machine();

            /* If dynamic port switching is enabled, we need to
             * put back the controller to default port once the
             * port comes back to idle */
            if ((UINT8)PORT_IDLE == port_[idx_port].get_state())
            {
                if ((is_dyanamic_port_switching_enabled())
                    && (idx_port == get_default_port())
#ifndef DISABLE_SMB_MASTER
                    && ((UINT8)MASTER_IDLE == get_master_state(instance_))
#endif
                    )
                {
                    //TRACE0(33, SMB_BASE, 0, "put port to default port");
                    I2CSMBx_PortSet((SMB_INSTANCE)instance_, idx_port);
                }
            }
            else
            {
                timerEventRequired = true;
            }
        }
    }
    return timerEventRequired;

}/* SMB_BASE::timer_task */


/******************************************************************************/
/** SMB_BASE::ber_isr function.
 * This function handles bus error portion of smbus interrupt service routine
 * This function needs to be called from isr whenever bus error is detected
 * again
 * @return None
 * @param None
 ******************************************************************************/
void SMB_BASE::ber_isr(void)
{
    uint8_t device;
    //TRACE0(34, SMB_BASE, 0, "ber_isr: enter ");
    uint8_t idx = 0;
    if (berCounter_!=0xFF)
    {
        berCounter_++;
    }
    idx =  I2CSMBx_PortGet((SMB_INSTANCE)instance_);
    port_[idx].set_bus_error_flag(true);

    I2CSMBx_MdoneDisable((SMB_INSTANCE)instance_);

    I2CSMBx_SdoneDisable((SMB_INSTANCE)instance_);

    I2CSMBx_ClrBERStatus((SMB_INSTANCE)instance_);

    /* Stop the DMA's */
#ifndef DISABLE_SMB_MASTER
    DMA_Stop((DMA_CHANNEL)((instance_*2)+1));
#endif
    DMA_Stop((DMA_CHANNEL)(instance_*2));

    I2CSMBx_FlushHwTxRxBuffers((SMB_INSTANCE)instance_);

    busErrorFlag_ = true;
}/* SMB_BASE::ber_isr */


/******************************************************************************/
/** SMB_BASE::handle_bus_error function.
 * This function handles bus error for the base. Basically it enables the base
 * again
 * @return None
 * @param None
 ******************************************************************************/
void SMB_BASE::handle_bus_error(void)
{
    //trace0(0, SMB_BASE, 0, "handle_bus_error: enter ");

    enable();
    if (is_timeouts_enabled())
    {
       I2CSMBx_TimeoutsEnable((SMB_INSTANCE)instance_, timeoutsFlag_);
    }
}/* SMB_BASE::handle_bus_error */

/******************************************************************************/
/** SMB_BASE::handle_master_wdt function.
 * This function handles master wdt for base. Whenever master wdt event happens
 * the base needs to take some action and this function takes care of that.
 * @return None
 * @param None
 ******************************************************************************/
#ifndef DISABLE_SMB_MASTER
void SMB_BASE::handle_master_wdt(void)
{
    uint8_t device;
    //TRACE0(35, SMB_BASE, 0, "handle_master_wdt: enter ");
    uint8_t idx =  I2CSMBx_PortGet((SMB_INSTANCE)instance_);
    port_[idx].set_bus_error_flag(true);

    I2CSMBx_MdoneDisable((SMB_INSTANCE)instance_);
    I2CSMBx_SdoneDisable((SMB_INSTANCE)instance_);

    /* Stop the DMA's */
    DMA_Stop((DMA_CHANNEL)((instance_*2)+1));

    DMA_Stop((DMA_CHANNEL)(instance_*2));

    I2CSMBx_FlushHwTxRxBuffers((SMB_INSTANCE)instance_);
    I2CSMBx_ClrAllCompletionStatus((SMB_INSTANCE)instance_);

    DMA_ChannelDeactivate((DMA_CHANNEL)((instance_*2)+1));
    
    DMA_ChannelDeactivate((DMA_CHANNEL)(instance_*2));

    /* Reset smbus controller */
    I2CSMBx_Reset((SMB_INSTANCE)instance_);

    //trace0(0, SMB_BASE, 0, "handle_master_wdt: Set and ClR MRUN, SRUN ");

    I2CSMBx_MRUNSet((SMB_INSTANCE)instance_);
    I2CSMBx_SRUNSet((SMB_INSTANCE)instance_);

    I2CSMBx_MRUNClr((SMB_INSTANCE)instance_);
    I2CSMBx_SRUNClr((SMB_INSTANCE)instance_);

}/* SMB_BASE::handle_master_wdt */
#endif

/******************************************************************************/
/** SMB_BASE::handle_bus_error_postprocessing_ports function.
 * This function handles bus error for the ports
 * @return None
 * @param None
 ******************************************************************************/
void SMB_BASE::handle_bus_error_postprocessing_ports(void)
{
    UINT8 idx_port;
    bool force_timeout_flag = true;

    //TRACE0(36, SMB_BASE, 0, "handle_bus_error_postprocessing_ports: enter ");

    I2CSMBx_SdoneDisable((SMB_INSTANCE)instance_);
    for (idx_port = SMB_PORT_0; idx_port < SMB_MAX_PORT_PER_CHANNEL; idx_port++)
    {
        if ( port_[idx_port].get_bus_error_flag())
        {
            if (mGET_BIT(sbit_CONFG_DISABLE_FORCE_TIMEOUT, config_))
            {
                force_timeout_flag = false;
            }
            port_[idx_port].enable(force_timeout_flag);
        }
        port_[idx_port].set_bus_error_flag(false);
    }
    I2CSMBx_SdoneEnable((SMB_INSTANCE)instance_);

    /* schedule timer task for executing port state machine */
    //kSET_EVENT_WAKETIMER(ms_to_ticks(SMBUS_TIMER_TICK_COUNT), smbus);

}/* SMB_BASE::handle_bus_error_postprocessing_ports */

#ifdef HANDLE_POWER_LOW_TO_HIGH_EVENT
/******************************************************************************/
/** SMB_BASE::handle_power_low_to_high_event function.
 * This function handles power low to high event. It basically forces a timeout
 * on the bus
 * @return TRUE if the event is acted upon, else FALSE
 * @param None
 ******************************************************************************/
bool SMB_BASE::handle_power_low_to_high_event(void)
{
    UINT8 idx_port;
    bool retVal = false;

    if ( ( is_clk_low_at_power_on_enabled()) &&
            (TRUE == powerLowToHighDetectFlag_))
    {
        if ( is_dyanamic_port_switching_enabled())
        {
            /* If dynamic port switching is enabled on the controller
             * all the associated ports needs a reset */
            for (idx_port = SMB_PORT_0; idx_port < SMB_MAX_PORT_PER_CHANNEL; idx_port++)
            {
                port_[idx_port].set_bus_error_flag(true);
            }
        }
        else
        {
            /* Reset the default port */
            idx_port = get_default_port();
            port_[idx_port].set_bus_error_flag(true);
        }

        handle_bus_error_postprocessing_ports();
        powerLowToHighDetectFlag_ = FALSE;
        retVal = true;
    }

    return retVal;

}/* SMB_BASE::handle_power_low_to_high_event */
#endif

/******************************************************************************/
/** SMB_BASE::is_need_to_disable function.
 * This function checks if smb needs to be disabled
  * @return TRUE/FALSE
 * @param None
 ******************************************************************************/
bool SMB_BASE::is_need_to_disable(void)
{
    bool retVal = false;

    if (disableFlag_)
    {
        //trace0(0, SMB_BASE, 0, "is_need_to_disable: disableFlag_ TRUE");
        disableFlag_ = false;

        /* If Enable bit is 0, disable smbus controller*/
        if (!(mGET_BIT(sbit_CONFG_ENABLE, config_)))
        {
            retVal = true;
        }
    }
    return retVal;
}/* SMB_BASE::is_need_to_disable */


/******************************************************************************/
/** SMB_BASE::get_port_state function.
 * This function returns the concerned port's state
 * @return port state
 * @param None
 ******************************************************************************/
UINT8 SMB_BASE::get_port_state(UINT8 idx_port) const
{

    if (idx_port >= SMB_MAX_PORT_PER_CHANNEL)
    {
        return PORT_NOT_PRESENT;
    }

    return port_[idx_port].get_state();

}/* SMB_BASE::get_port_state */

/** @}
*/
