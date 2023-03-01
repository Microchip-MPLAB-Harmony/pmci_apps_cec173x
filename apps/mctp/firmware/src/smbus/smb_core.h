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
/** @defgroup smb smb_core
 *  @{
 */
/** @file smb_core.hpp
 \brief the smb core hpp file

<b>Platform:</b> This is ARM-based component
  Details of hardware blocks on which this smbus reuse component is tested:
    smbus HW Block name - blk_i2c_spb, smbus HW Block build # - B0076   
    dma HW Block name - blk_dma_ahb, dma HW Block build # - B0007
    FPGA build used for testing -> MEC1322 FPGA build Rev-81-Randy - 081_chip_top_fpga2_run1
    //depotAE/projects/MEC1322/FPGA builds/Rev-81-Randy/081_chip_top_fpga2_run1.mcs

<b>Toolset:</b> MDK-Lite V 4.23; Armcc, Armasm, ArmLink, ArmAr, FromElf - V4.1.0.894 
<b>Reference:</b> smsc_reusable_fw_requirement.doc */

/*******************************************************************************
 $File: //depot_pcs/FWEng/solutions/Glacier_GEN3/maincodeline/Harmony/pmci_stack/pmci_apps_cec173x/apps/mctp/firmware/src/smbus/smb_core.h $

Version Control Information (Perforce)
*******************************************************************************
$Revision: #3 $
$DateTime: 2023/02/24 06:07:07 $
$Author: i64652 $
  Change Description:
    2. Code changes to support Master Tx chaining
    1. Branched from //depot_pcs/FWEng/projects/MEC1324/maincodeline/libSmbus/source/smb/ 
        label - MEC1324_LIB_SMBUS_0400
*******************************************************************************/
#ifndef SMB_CORE_H_
#define SMB_CORE_H_

void smbus_core_init_task(void);
void smbus_interrupt_task(void);
void smbus_timer_task(void);
bool is_bus_error_flag_set(const UINT8 channel);
void smb_disable(const UINT8 channel);
void smb_enable(const UINT8 channel);
void smb_core_isr(const UINT8 channel);
void smb_core_isr_dma(const UINT8 channel, const bool slave_dma_flag);
#ifndef DISABLE_SMB_MASTER
void smb_core_master_wdt(const UINT8 channel);
void smb_update_master_wdt_interval(const UINT8 channel,const UINT8 wdt_interval);
UINT8 check_master_availablity(const UINT8 channel, const bool defaultPortFlag, UINT8 port);
bool execute_master_request(const UINT8 channel, UINT8 *buffer_ptr, const UINT8 smb_protocol, const UINT8 writeCount, 
     const bool pecEnable, MASTER_FUNC_PTR func_ptr, const bool blockingFlag, const bool readChainedFlag, const bool writeChainedFlag);
UINT8 get_master_state(const UINT8 channel);
#endif
void smb_update_nack_mechanism(const UINT8 channel,const UINT8 nackMechanism);
void smb_update_ber_counter_value(const UINT8 channel,const UINT8 value);
void smb_update_clk_value(const UINT8 channel,const UINT16 clk_value);
void smb_update_timeoutScaling_value(const UINT8 channel,const UINT32 value);
void smb_config_change_event(const UINT8 channel,const UINT8 configValue,const UINT8 cfgState);
void smb_speed_config_change_event(UINT8 channel, UINT8 configValue); //VB
void smb_port_config_change_event(const UINT8 channel,const UINT8 configValue);
void smb_own_address_changed_event(const UINT8 channel, const UINT16 value);
void smb_port_config_change(const UINT8 channel,const UINT8 portConfigValue);
void smb_set_default_port(const UINT8 channel,const UINT8 port);
void set_speed(UINT8 channel, UINT8 speed);
#ifndef DISABLE_ALL_SLAVE_BUFFERS
void smbus_event_task(void);
UINT8 smb_core_register_slave(const UINT8 channel, SLAVE_FUNC_PTR slaveFuncPtr);
UINT8 smb_core_deregister_slave(const UINT8 channel, SLAVE_FUNC_PTR slaveFuncPtr);
#endif
UINT8 change_port(const UINT8 channel, const UINT8 port);
#ifdef HANDLE_POWER_LOW_TO_HIGH_EVENT
void smb_power_low_to_high_event_set(void);
#endif

/* Events that smbus driver thread waits on (event_flags_2_smbus) */
#define SMBUS_EVENT_ISR BIT(0)
#define SMBUS_EVENT_TIMER   BIT(1)
#define SMBUS_EVENT_MONITOR_WDT BIT(2)
#define SMBUS_EVENT_MSG_QUEUE BIT(3)
#define SMBUS_EVENT_RESERVED1 BIT(4)

/* Other events (event_flags_2_smbus) */
//SMBUS MASTER BLOCKING REQ FLAGS, one for each controller
#define SMBUS_EVENT_MASTER_BLOCKING_REQ1 BIT(4) 
#define SMBUS_EVENT_MASTER_BLOCKING_REQ2 BIT(5)
#define SMBUS_EVENT_MASTER_BLOCKING_REQ3 BIT(6)
#define SMBUS_EVENT_MASTER_BLOCKING_REQ4 BIT(7)

#endif /* SMB_CORE_H_ */
/** @}
*/
