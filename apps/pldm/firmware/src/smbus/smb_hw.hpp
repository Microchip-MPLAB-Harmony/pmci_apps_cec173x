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
*****************************************************************************
 *  @{
 */
/** @file smb_hw.hpp
 \brief the smb hw hpp file
This file implements the HW_SMB class. This class encapsulates the SMB
hardware MMCR’s and all functions related to programming them. Whenever
any layer needs to program the MMCR’s it will go through object of this class.
The data members of the object of this class needs to be statically created
in the memory mapped address of the SMB MMCRs.

<b>Platform:</b> This is ARM-based component
  Details of hardware blocks on which this smbus reuse component is tested:
    smbus HW Block name - blk_i2c_spb, smbus HW Block build # - B0076   
    dma HW Block name - blk_dma_ahb, dma HW Block build # - B0007
    FPGA build used for testing -> MEC1322 FPGA build Rev-81-Randy - 081_chip_top_fpga2_run1
    //depotAE/projects/MEC1322/FPGA builds/Rev-81-Randy/081_chip_top_fpga2_run1.mcs

<b>Toolset:</b> MDK-Lite V 4.23; Armcc, Armasm, ArmLink, ArmAr, FromElf - V4.1.0.894 
<b>Reference:</b> smsc_reusable_fw_requirement.doc */

/*******************************************************************************
 $File: //depot_pcs/FWEng/solutions/cec1712_GEN2/maincodeline/secureboot_app/src/plib/smbus/smb_hw.hpp $

Version Control Information (Perforce)
*******************************************************************************
$Revision: #3 $
$DateTime: 2020/04/09 08:29:34 $
$Author: I19961 $
  Change Description:
    2. Code changes to support Master Tx chaining
    1. Branched from //depot_pcs/FWEng/projects/MEC1324/maincodeline/libSmbus/source/smb/ 
        label - MEC1324_LIB_SMBUS_0400
*******************************************************************************/
#ifndef SMB_HW_HPP_
#define SMB_HW_HPP_

#include "smb_config.h"

namespace SMB{

enum configuration_reg_bits
{
    sbit_CONFIGURATION_TCEN = BIT_4_MASK
   ,sbit_CONFIGURATION_SLWCLK = BIT_5_MASK
   ,sbit_CONFIGURATION_PECEN = BIT_7_MASK
   ,sbit_CONFIGURATION_DFEN = BIT_8_MASK
   ,sbit_CONFIGURATION_RESET = BIT_9_MASK
   ,sbit_CONFIGURATION_ENABLE = BIT_10_MASK
   ,sbit_CONFIGURATION_DSA = BIT_11_MASK
   ,sbit_CONFIGURATION_FAIR = BIT_12_MASK
   ,sbit_CONFIGURATION_DISABLE_SLAVE = BIT_13_MASK
   ,sbit_CONFIGURATION_DISABLE_GENERAL_CALL = BIT_14_MASK
   ,sbit_CONFIGURATION_FLUSH_SXBUF = BIT_16_MASK
   ,sbit_CONFIGURATION_FLUSH_SRBUF = BIT_17_MASK
   ,sbit_CONFIGURATION_FLUSH_MXBUF = BIT_18_MASK
   ,sbit_CONFIGURATION_FLUSH_MRBUF =    BIT_19_MASK
   ,sbit_CONFIGURATION_ENIDI = BIT_29_MASK
   ,sbit_CONFIGURATION_ENMI = BIT_30_MASK
   ,sbit_CONFIGURATION_ENSI = BIT_31_MASK
};

enum configuration_reg_byte3_bits
{
    sbit_CONFIGURATION_BYTE3_ENIDI = BIT_5_MASK
   ,sbit_CONFIGURATION_BYTE3_ENMI = BIT_6_MASK
   ,sbit_CONFIGURATION_BYTE3_ENSI = BIT_7_MASK
};

/* Completion Register's bits */
enum completion_reg_bits
{
    sbit_COMPLETION_DTEN            = BIT_2_MASK
    ,sbit_COMPLETION_MCEN           = BIT_3_MASK
    ,sbit_COMPLETION_SCEN           = BIT_4_MASK
    ,sbit_COMPLETION_BIDEN          = BIT_5_MASK
    ,sbit_COMPLETION_TIMERR         = BIT_6_MASK
    ,sbit_COMPLETION_DTO            = BIT_8_MASK
    ,sbit_COMPLETION_MCTO           = BIT_9_MASK
    ,sbit_COMPLETION_SCTO           = BIT_10_MASK
    ,sbit_COMPLETION_CHDL           = BIT_11_MASK
    ,sbit_COMPLETION_CHDH           = BIT_12_MASK
    ,sbit_COMPLETION_BER            = BIT_13_MASK
    ,sbit_COMPLETION_LAB            = BIT_14_MASK
    ,sbit_COMPLETION_SNAKR          = BIT_16_MASK
    ,sbit_COMPLETION_STR            = BIT_17_MASK
    ,sbit_COMPLETION_SPROT          = BIT_19_MASK
    ,sbit_COMPLETION_REPEAT_READ    = BIT_20_MASK
    ,sbit_COMPLETION_REPEAT_WRITE   = BIT_21_MASK
    ,sbit_COMPLETION_MNAKX          = BIT_24_MASK
    ,sbit_COMPLETION_MTR            = BIT_25_MASK
    ,sbit_COMPLETION_IDLE           = BIT_29_MASK
    ,sbit_COMPLETION_MDONE          = BIT_30_MASK
    ,sbit_COMPLETION_SDONE          = BIT_31_MASK
};

enum completion_reg_byte3_bits
{
     SMB_SDONE_POSITION_IN_BYTE             = BIT_7_MASK
    ,SMB_MDONE_POSITION_IN_BYTE             = BIT_6_MASK
    ,SMB_IDLE_POSITION_IN_BYTE              = BIT_5_MASK
    ,SMB_MNAKX_POSITION_IN_BYTE             = BIT_0_MASK
};

enum completion_reg_byte2_bits
{
    SMB_SNAKR_POSITION_IN_BYTE              = BIT_0_MASK
    ,SMB_SPROT_POSITION_IN_BYTE             = BIT_3_MASK
    ,SMB_REPEAT_READ_POSITION_IN_BYTE       = BIT_4_MASK
    ,SMB_REPEAT_WRITE_POSITION_IN_BYTE      = BIT_5_MASK
};
enum completion_reg_byte1_bits
{
    SMB_LAB_POSITION_IN_BYTE                = BIT_6_MASK
    ,SMB_BER_POSITION_IN_BYTE               = BIT_5_MASK
};

enum completion_reg_byte0_bits
{
    SMB_DTEN_POSITION_IN_BYTE               = BIT_2_MASK
    ,SMB_MCEN_POSITION_IN_BYTE              = BIT_3_MASK
    ,SMB_SCEN_POSITION_IN_BYTE              = BIT_4_MASK
    ,SMB_BIDEN_POSITION_IN_BYTE             = BIT_5_MASK
    ,SMB_TIMERR_POSITION_IN_BYTE            = BIT_6_MASK
};

enum master_command_bits
{
    sbit_MASTER_COMMAND_MRUN = BIT_0_MASK
   ,sbit_MASTER_COMMAND_MPROCEED = BIT_1_MASK
};

enum slave_command_bits
{
    sbit_SLAVE_COMMAND_SRUN = BIT_0_MASK
   ,sbit_SLAVE_COMMAND_SPROCEED = BIT_1_MASK
};

typedef struct _BYTE_ACCESS_
{
    VUINT8 BYTE0;
    VUINT8 BYTE1;
    VUINT8 BYTE2;
    VUINT8 BYTE3;
}BYTE_ACCESS;

class HW_SMB{

    //private:
    public:

    //Control/Status register offset=00h
        union{
            VUINT8  REG8_CONTROL_STATUS;
            VUINT32 dword_CONTROL_STATUS;
        };

    //Own Address Register offset=04h
        union{
            VUINT16  REG16_OWN_ADDRESS;
            VUINT32  dword_OWN_ADDRESS;
        };

    //Data Register offset=08h
        union{
            VUINT8  REG8_DATA_REG;
            VUINT32 dword_SMB_DATA_REG;
        };

    //SMBus Master Command Register offset=0Ch
        union{
            BYTE_ACCESS REG32_MASTER_CMD_BYTES;
            VUINT32 REG32_MASTER_CMD;
        };

    //SMBus Slave Command Register offset=10h
        union{
            BYTE_ACCESS REG32_SLAVE_CMD_BYTES;
            VUINT32 REG32_SLAVE_CMD;
        };

    //PEC Register offset=14h
        union{
            VUINT8  REG8_PEC;
            VUINT32 dword_PEC;
        };

    //Data Timing 2 Register offset=18h
        union{
            VUINT8  REG8_DATA_TIMING2;
            VUINT32 dword_DATA_TIMING2;
        };

    //Reserved register offset=1Ch
        VUINT32 reserved2;

    //Completion Register offset=20h
        union{
            BYTE_ACCESS REG32_COMPLETION_BYTES;
            VUINT32 REG32_COMPLETION;
        };

    //Idle Scaling Register offset=24h
        VUINT32 REG32_IDLE_SCALING;

    //Configuration Register offset=28h
        union{
            BYTE_ACCESS REG32_CONFIGURATION_BYTES;
            VUINT32 REG32_CONFIGURATION;
        };

    //Bus Clock Register offset=2Ch
        union{
            VUINT16  REG16_BUS_CLOCK;
            VUINT32  dword_BUS_CLOCK;
        };

    //Block ID Register offset=30h
        union{
            VUINT8  REG8_BLOCK_ID;
            VUINT32 dword_SMB_BLOCK_ID;
        };

    //Revision Register offset=34h
        union{
            VUINT8  REG8_REVISION;
            VUINT32 dword_REVISION;
        };

    //Bit-Bang Control Register offset=38h
        union{
            VUINT8  REG8_BITBANG_CTRL;
            VUINT32 dword_BITBANG_CTRL;
        };

    //SMSC Reserved Register offset=3Ch
        union{
            VUINT8  REG8_SMSC_RESERVED;
            VUINT32 dword_SMSC_RESERVED;
        };

    //Data Timing Register offset=40h
        VUINT32 REG32_DATA_TIMING;

    //Time-Out Scaling Register offset=44h
        VUINT32 REG32_TIMEOUT_SCALING;

    //SMBus Slave Transmit Buffer Register offset=48h
        union{
            VUINT8  REG8_SLAVE_TX_BUF;
            VUINT32 dword_SLAVE_TX_BUF;
        };

    //SMBus Slave Receive Buffer Register offset=4Ch
        union{
            VUINT8  REG8_SLAVE_RX_BUF;
            VUINT32 dword_SLAVE_RX_BUF;
        };

    //SMBus Master Transmit Buffer Register offset=50h
        union{
            VUINT8  REG8_MASTER_TX_BUF;
            VUINT32 dword_MASTER_TX_BUF;
        };

    //SMBus Master Receive Buffer Register offset=54h
        union{
            VUINT8  REG8_MASTER_RX_BUF;
            VUINT32 dword_MASTER_RX_BUF;
        };

        HW_SMB(const HW_SMB& r); //no copy
        HW_SMB& operator=(const HW_SMB& r);  //no assignment

    //public:

        HW_SMB(){};
        //~HW_SMB(){};

        void reset(void);
        void enable(const UINT16 ownAddress, const UINT8 port,
                const UINT8 speed, const bool fairnessEnableFlag);
        void start_slave(const UINT8 read_count, const UINT8 write_count, const bool pecFlag);
        void timing_init(const UINT8 speed, const bool fairnessEnable);

        void flush_hw_tx_rx_buffers(void);

        /******************************************************************************/
        /** start_master function.
         * This function starts the master state machine
         * @return master_command - master command value
         * @param None
        *******************************************************************************/
        void start_master(const UINT32 master_command)
        {
            /* Program the Master Command Register */
            REG32_MASTER_CMD = (master_command |  sbit_MASTER_COMMAND_MPROCEED
                                                    | sbit_MASTER_COMMAND_MRUN);
        }/* start_master */
        
        /******************************************************************************/
        /** program_masterCommandReg_readcount function.
         * This function programs the readcount in the master command register
         * @return readCount - readcount value
         * @param None
        *******************************************************************************/
        void program_masterCommandReg_readcount(const UINT8 readCount)
        {           
            /* Program the Read count in the Master Command Register */
            REG32_MASTER_CMD_BYTES.BYTE3 = readCount;                   
            
        }/* program_masterCommandReg_readcount */
        
        /******************************************************************************/
        /** program_masterCommandReg_writecount function.
         * This function programs the writeCount in the master command register
         * @return writeCount - writeCount value
         * @param None
        *******************************************************************************/
        void program_masterCommandReg_writecount(const UINT8 writeCount)
        {           
            /* Program the Write count in the Master Command Register */
            REG32_MASTER_CMD_BYTES.BYTE2 = writeCount;                  
            
        }/* program_masterCommandReg_readcount */

        /******************************************************************************/
        /** master_proceed function.
         * This function sets the PROCEED bit in master command register
         * @param None
         * @return None
        *******************************************************************************/
        void master_proceed(void)
        {
            /* Set the Proceed bit in the Master Command Register */
            REG32_MASTER_CMD = REG32_MASTER_CMD | sbit_MASTER_COMMAND_MPROCEED;
        }/* master_proceed */

        /******************************************************************************/
        /** master_run function.
         * This function sets the PROCEED bit and MRUN in master command register
         * @param None
         * @return None
        *******************************************************************************/
        void master_run(void)
        {
            REG32_MASTER_CMD |= (sbit_MASTER_COMMAND_MPROCEED | sbit_MASTER_COMMAND_MRUN);
        }/* master_run  */

        /******************************************************************************/
        /** set_master_command function.
         * This function programs the master command register value
         * @param cmdValue Master command value
         * @return None
        *******************************************************************************/
        void set_master_command(const UINT8 cmdValue)
        {
            REG32_MASTER_CMD = cmdValue;
        }/* set_master_command */

        /******************************************************************************/
        /** mdone_enable function.
         * This function enables MDONE interrupt
         * @return None
         * @param None
        *******************************************************************************/
        void mdone_enable(void)
        {
            mSET_BIT(sbit_CONFIGURATION_ENMI, REG32_CONFIGURATION);
        }/* mdone_enable */

        /******************************************************************************/
        /** mdone_disable function.
         * This function disables MDONE interrupt
         * @return None
         * @param None
        *******************************************************************************/
        void mdone_disable(void)
        {
            mCLR_BIT(sbit_CONFIGURATION_ENMI, REG32_CONFIGURATION);
        }/* mdone_disable */

        /******************************************************************************/
        /** sdone_enable function.
         * This function enables SDONE interrupt
         * @return None
         * @param None
        *******************************************************************************/
        void sdone_enable(void)
        {
            mSET_BIT(sbit_CONFIGURATION_ENSI, REG32_CONFIGURATION);
        }/* sdone_enable  */

        /******************************************************************************/
        /** sdone_disable function.
         * This function disables SDONE interrupt
         * @return None
         * @param None
        *******************************************************************************/
        void sdone_disable(void)
        {
            mCLR_BIT(sbit_CONFIGURATION_ENSI, REG32_CONFIGURATION);
        }/* sdone_disable */

        /******************************************************************************/
        /** set_own_slave_address function.
         * This function sets the own address in the own address register
         * @param value - the own address value
         * @return None
        *******************************************************************************/
        void set_own_slave_address(const UINT16 value)
        {
            REG16_OWN_ADDRESS = value;
        }/* set_own_slave_address */

        /******************************************************************************/
        /** set_clk_value function.
         * This function sets the clock value in the Bus clk register
         * @param value - the clock value
         * @return None
        *******************************************************************************/
        void set_clk_value(const UINT16 value)
        {
            REG16_BUS_CLOCK = value;
        }/* set_clk_value */

        /******************************************************************************/
        /** set_clk_value function.
         * This function sets the value in the Timeout Scaling Register
         * @param value - the timeout Scaling value
         * @return None
        *******************************************************************************/
        void set_timeoutScaling_value(const UINT32 value)
        {
            REG32_TIMEOUT_SCALING = value;
        }/* set_timeoutScaling_value */

        /******************************************************************************/
        /** set_dataTiming2_value function.
         * This function sets the value in the Data Timing 2 Register
         * @param value - the timeout Scaling value
         * @return None
        *******************************************************************************/
        void set_dataTiming2_value(const UINT32 value)
        {
            REG8_DATA_TIMING2 = value;
        }/* set_dataTiming2_value */

        /******************************************************************************/
        /** enable_timeouts function.
         * This function enables timeouts
         * @param None
         * @return None
        *******************************************************************************/
        void enable_timeouts(const UINT8 timeoutsFlag)
        {
            REG32_COMPLETION_BYTES.BYTE0 = 0;
            
            REG32_COMPLETION_BYTES.BYTE0 |= timeoutsFlag;

            REG32_CONFIGURATION |= sbit_CONFIGURATION_TCEN;
        }

        /******************************************************************************/
        /** set_port function.
         * This function sets the smbus port
         * @param port the port to configure
         * @return None
        *******************************************************************************/
        void set_port(const UINT8 port)
        {
            REG32_CONFIGURATION_BYTES.BYTE0 = ((REG32_CONFIGURATION_BYTES.BYTE0 & 0xF0) | port);
            //TRACE1(1091, SMB_HW, 0, "set_port: SMB32_REG_CONFIGURATION = %02Xh", REG32_CONFIGURATION_BYTES.BYTE0);
        }/* set_port */

        /******************************************************************************/
        /** get_port function.
         * This function returns the port that is configured
         * @param None
         * @return port - port that is configured
        *******************************************************************************/
        UINT8 get_port(void)
        {
            return (UINT8)(REG32_CONFIGURATION_BYTES.BYTE0 & 0xFu);

        }/* get_port */


        /******************************************************************************/
        /** is_bus_error_set function.
         * This function returns TRUE if BER bit is set, else FALSE
         * @param None
         * @return TRUE/FALSE
        *******************************************************************************/
        bool is_bus_error_set(void)
        {
            return static_cast<bool>(mGET_BIT(sbit_COMPLETION_BER, REG32_COMPLETION));

        }/* is_bus_error_set */

        /******************************************************************************/
        /** is_mdone_set function.
         * This function return TRUE if MDONE is set, else FALSE
         * @return TRUE/FALSE
         * @param None
        *******************************************************************************/
        bool is_mdone_set(void)
        {
            return static_cast<bool> ( (mGET_BIT(sbit_COMPLETION_MDONE, REG32_COMPLETION)) &&
                            (mGET_BIT(sbit_CONFIGURATION_ENMI, REG32_CONFIGURATION)));

        }/* is_mdone_set */

        /******************************************************************************/
        /** is_sdone_set function.
         * This function returns TRUE if SDONE bit is set, else FALSE
         * @return None
         * @param None
        *******************************************************************************/
        bool is_sdone_set(void)
        {
            return static_cast<bool> ( (mGET_BIT(sbit_COMPLETION_SDONE, REG32_COMPLETION)) &&
                            (mGET_BIT(sbit_CONFIGURATION_ENSI, REG32_CONFIGURATION)));

        }/* is_sdone_set  */

        /******************************************************************************/
        /** is_LAB_set function.
         * This function returns TRUE if LAB bit is set, else FALSE
         * @return TRUE/FALSE
         * @param None
        *******************************************************************************/
        bool is_LAB_set(void)
        {
            return static_cast<bool> ( mGET_BIT(sbit_COMPLETION_LAB, REG32_COMPLETION));
        }/* is_LAB_set */

        /******************************************************************************/
        /** is_MNAKX_set function.
         * This function returns TRUE if MNAKX (master nack) bit is set, else FALSE
         * @return TRUE/FALSE
         * @param None
        *******************************************************************************/
        bool is_MNAKX_set(void)
        {
            return static_cast<bool> ( mGET_BIT(sbit_COMPLETION_MNAKX, REG32_COMPLETION));

        }/* is_MNAKX_set */

        /******************************************************************************/
        /** is_SNAKR_set function.
         * This function returns TRUE if SNAKR (slave nack) bit is set, else FALSE
         * @return TRUE/FALSE
         * @param None
        *******************************************************************************/
        bool is_SNAKR_set(void)
        {
            return static_cast<bool> ( mGET_BIT(sbit_COMPLETION_SNAKR, REG32_COMPLETION));

        }/* is_SNAKR_set */

        /******************************************************************************/
        /** is_STR_set function.
         * This function returns TRUE if STR bit is set, else FALSE
         * @return None
         * @param None
        *******************************************************************************/
        bool is_STR_set(void)
        {
            return static_cast<bool> ( mGET_BIT(sbit_COMPLETION_STR, REG32_COMPLETION));

        }/* is_STR_set */

        /******************************************************************************/
        /** is_REPEAT_WRITE_set function.
         * This function returns TRUE if REPEAT_WRITE bit is set, else FALSE
         * @return TRUE/FALSE
         * @param None
        *******************************************************************************/
        bool is_REPEAT_WRITE_set(void)
        {
            return static_cast<bool> ( mGET_BIT(sbit_COMPLETION_REPEAT_WRITE, REG32_COMPLETION));

        }/* is_REPEAT_WRITE_set */

        /******************************************************************************/
        /** is_REPEAT_READ_set function.
         * This function returns TRUE if REPEAT_READ bit is set, else FALSE
         * @return TRUE/FALSE
         * @param None
        *******************************************************************************/
        bool is_REPEAT_READ_set(void)
        {
            return static_cast<bool> ( mGET_BIT(sbit_COMPLETION_REPEAT_READ, REG32_COMPLETION));

        }/* is_REPEAT_READ_set */

        /******************************************************************************/
        /** is_MTR_set function.
         * This function returns TRUE if MTR (master xmit) bit is set, else FALSE
         * @return TRUE/FALSE
         * @param None
        *******************************************************************************/
        bool is_MTR_set(void)
        {
            return static_cast<bool> ( mGET_BIT(sbit_COMPLETION_MTR, REG32_COMPLETION));

        }/* is_MTR_set */

        /******************************************************************************/
        /** is_timer_error_set function.
         * This function returns TRUE if TIMERR bit is set, else FALSE
         * @return TRUE/FALSE
         * @param None
        *******************************************************************************/
        bool is_timer_error_set(void)
        {
            return static_cast<bool> ( mGET_BIT(sbit_COMPLETION_TIMERR, REG32_COMPLETION));

        }/* is_timer_error_set */


        /******************************************************************************/
        /** set_MRUN function.
         * This function sets the MRUN bit
         * @return None
         * @param None
        *******************************************************************************/
        void set_MRUN(void)
        {
            mSET_BIT(sbit_MASTER_COMMAND_MRUN, REG32_MASTER_CMD);
        }/* set_MRUN */

        /******************************************************************************/
        /** clr_MRUN function.
         * This function clears MRUN bit
         * @return None
         * @param None
        *******************************************************************************/
        void clr_MRUN(void)
        {
            mCLR_BIT(sbit_MASTER_COMMAND_MRUN, REG32_MASTER_CMD);
        }/* clr_MRUN */

        /******************************************************************************/
        /** set_SRUN function.
         * This function sets SRUN bit
         * @return None
         * @param None
        *******************************************************************************/
        void set_SRUN(void)
        {
            mSET_BIT(sbit_SLAVE_COMMAND_SRUN, REG32_SLAVE_CMD);
        }/* set_SRUN */

        /******************************************************************************/
        /** clr_SRUN function.
         * This function clears the SRUN bit
         * @return None
         * @param None
        *******************************************************************************/
        void clr_SRUN(void)
        {
            mCLR_BIT(sbit_SLAVE_COMMAND_SRUN, REG32_SLAVE_CMD);
        }/* clr_SRUN */


        /******************************************************************************/
        /** timer_task function.
         * This function initializes the smbus base
         * @return None
         * @param None
        *******************************************************************************/
        void clr_all_completion_status(void)
        {
            clr_LAB_completion_status();
            clr_BER_status();
            clr_master_completion_status();
            clr_slave_completion_status();
        }/*  */

        /******************************************************************************/
        /** clr_master_completion_status function.
         * This function clears the master completion status bits
         * @return None
         * @param None
        *******************************************************************************/
        void clr_master_completion_status(void)
        {
            REG32_COMPLETION_BYTES.BYTE3 = (SMB_MDONE_POSITION_IN_BYTE |
                                                SMB_MNAKX_POSITION_IN_BYTE);
        }/* clr_master_completion_status */

        /*****************************************************************************/
        /** clr_LAB_completion_status function.
         * This function clears the LAB bit in the completion register
         * @return None
         * @param None
        *******************************************************************************/
        void clr_LAB_completion_status(void)
        {
            REG32_COMPLETION_BYTES.BYTE1 = SMB_LAB_POSITION_IN_BYTE;
        }/* clr_LAB_completion_status */

        /******************************************************************************/
        /** clr_slave_completion_status function.
         * This function clears all slave completion status
         * @return None
         * @param None
        *******************************************************************************/
        void clr_slave_completion_status(void)
        {
            REG32_COMPLETION_BYTES.BYTE3 = SMB_SDONE_POSITION_IN_BYTE;
            REG32_COMPLETION_BYTES.BYTE2 = (SMB_SNAKR_POSITION_IN_BYTE |
                                                      SMB_SPROT_POSITION_IN_BYTE |
                                                    SMB_REPEAT_READ_POSITION_IN_BYTE |
                                                    SMB_REPEAT_WRITE_POSITION_IN_BYTE);
        }/* clr_slave_completion_status */

        /******************************************************************************/
        /** clr_BER_status function.
         * This function clears BER status bit in completion register
         * @return None
         * @param None
        *******************************************************************************/
        void clr_BER_status(void)
        {
            REG32_COMPLETION_BYTES.BYTE1 = SMB_BER_POSITION_IN_BYTE;
        }/* clr_BER_status */

        /******************************************************************************/
        /** flush_slave_tx_buffer function.
         * This function clears the slave tx buffer
         * @return None
         * @param None
        *******************************************************************************/
        void flush_slave_tx_buffer(void)
        {
            mSET_BIT(sbit_CONFIGURATION_FLUSH_SXBUF, REG32_CONFIGURATION);
        }/* flush_slave_tx_buffer */

        /******************************************************************************/
        /** flush_slave_rx_buffer function.
         * This function clears the slave rx buffer
         * @return None
         * @param None
        *******************************************************************************/
        void flush_slave_rx_buffer(void)
        {
            mSET_BIT(sbit_CONFIGURATION_FLUSH_SRBUF, REG32_CONFIGURATION);
        }/* flush_slave_rx_buffer */

        /******************************************************************************/
        /** flush_master_tx_buffer function.
         * This function clears the master tx buffer
         * @return None
         * @param None
        *******************************************************************************/
        void flush_master_tx_buffer(void)
        {
            mSET_BIT(sbit_CONFIGURATION_FLUSH_MXBUF, REG32_CONFIGURATION);
        }/* flush_master_tx_buffer */

        /******************************************************************************/
        /** flush_master_rx_buffer function.
         * This function clears the master rx buffer
         * @return None
         * @param None
        *******************************************************************************/
        void flush_master_rx_buffer(void)
        {
            mSET_BIT(sbit_CONFIGURATION_FLUSH_MRBUF, REG32_CONFIGURATION);
        }/* flush_master_rx_buffer */

        /******************************************************************************/
        /** get_slave_read_count function.
         * This function return read count programmed in slave command register
         * @return None
         * @param None
        *******************************************************************************/
        UINT8 get_slave_read_count(void)
        {
            return (UINT8)REG32_SLAVE_CMD_BYTES.BYTE2;
        }/* get_slave_read_count */

        /******************************************************************************/
        /** get_master_write_count function.
         * This function returns the write count programmed in the master command register
         * @return None
         * @param None
        *******************************************************************************/
        UINT8 get_master_write_count(void)
        {
            return (UINT32)REG32_MASTER_CMD_BYTES.BYTE2;
        }/* get_master_write_count */

        /******************************************************************************/
        /** get_master_read_count function.
         * This function returns the read count programmed in the master command register
         * @return None
         * @param None
        *******************************************************************************/
        UINT8 get_master_read_count(void)
        {
            return (UINT32)REG32_MASTER_CMD_BYTES.BYTE3;
        }/* get_master_read_count  */

        /******************************************************************************/
        /** get_slave_rx_buffer_address function.
         * This function returns the address of slave rx buffer
         * @return None
         * @param None
        *******************************************************************************/
        UINT32 get_slave_rx_buffer_address(void)
        {
            return (UINT32)(&REG8_SLAVE_RX_BUF);
        }/* get_slave_rx_buffer_address  */

        /******************************************************************************/
        /** get_slave_tx_buffer_address function.
         * This function returns the address of slave tx buffer
         * @return None
         * @param None
        *******************************************************************************/
        UINT32 get_slave_tx_buffer_address(void)
        {
            return (UINT32)(&REG8_SLAVE_TX_BUF);
        }/* get_slave_tx_buffer_address */

        /******************************************************************************/
        /** get_master_rx_buffer_address function.
         * This function returns the address of master rx buffer
         * @return None
         * @param None
        *******************************************************************************/
        UINT32 get_master_rx_buffer_address(void)
        {
            return (UINT32)(&REG8_MASTER_RX_BUF);
        }/* get_master_rx_buffer_address */

        /******************************************************************************/
        /** get_master_tx_buffer_address function.
         * This function returns the address of master tx buffer
         * @return None
         * @param None
        *******************************************************************************/
        UINT32 get_master_tx_buffer_address(void)
        {
            return (UINT32)(&REG8_MASTER_TX_BUF);
        }/* get_master_tx_buffer_address */

        /**************************************************************************************/


    }; /* class HW_SMB */

} /* namespace SMB */

#endif /* SMB_HW_HPP_ */
/** @}
*/
