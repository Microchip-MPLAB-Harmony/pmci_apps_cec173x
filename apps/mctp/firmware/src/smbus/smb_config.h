/*****************************************************************************
* Copyright (c) 2020 Microchip Technology Inc. and its subsidiaries.
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
/** @file smb_cfg_cec173x.h
 \brief the smb config file
 This file needs to updated based on the configuration required for the smbus
 driver running on CEC173x SoC.
*******************************************************************************/

#ifndef SMB_CFG_CEC173X_H_
#define SMB_CFG_CEC173X_H_

/* CEC173x info */
//#define GPIO_BASE_ADDRESS 0x40081000UL
#define GPIO_PARIN_BASE_ADDRESS (GPIO_BASE_ADDRESS + 0x0300UL)
#define GPIO_PAROUT_BASE_ADDRESS (GPIO_BASE_ADDRESS + 0x0380UL)
#define GPIO_CTRL_REG_ADDRESS(n) (GPIO_BASE_ADDRESS + ((uint32_t)(n) * 4UL))
#define GPIO_PIN_REG_ADDRESS(n) (GPIO_PARIN_BASE_ADDRESS + ((uint32_t)(n) * 4UL))
#define GPIO_POUT_REG_ADDRESS(n) (GPIO_PARIN_BASE_ADDRESS + ((uint32_t)(n) * 4UL))

//#define SMB0_BASE_ADDRESS 0x40004000UL
//#define SMB1_BASE_ADDRESS 0x40004400UL
//#define SMB2_BASE_ADDRESS 0x40004800UL
//#define SMB3_BASE_ADDRESS 0x40004C00UL
//#define SMB4_BASE_ADDRESS 0x40005000UL
//#define DMA_MAIN_BASE_ADDRESS 0x40002400UL
#define DMA_CHAN_SPACING 0x40UL
#define DMA_CHAN_BASE_ADDR(n)                                          \
    (DMA_MAIN_BASE_ADDRESS + (((uint32_t)(n) + 1U) * DMA_CHAN_SPACING))

/*
 * Use GIRQ number from chip documentation, let the SoC/peripheral
 * library handle translation.
 */
#define MCHP_SMB_GIRQ 13U
#define MCHP_DMA_GIRQ 14U


#define SMB_GIRQ_ID     MEC_GIRQ13_ID
#define DMA_GIRQ_ID     MEC_GIRQ14_ID


//User configurable section
//----------------------------------------------------------------------
/* Maximum number of smbus controller */
#define MAX_SMB 5
/* Maximum number of ports per SMBus controller */
#define SMB_MAX_PORT_PER_CHANNEL    16u

// SLAVE CONFIGURATIONS
/* Define this macro to disable all slave buffers, slave will be enabled only to
 * configure for NACK */
//#define DISABLE_ALL_SLAVE_BUFFERS

/* Define this macro to disable pending buffers logic */
#define DISABLE_PENDING_PACKETS

/* Maximum number of buffers per slave controller */
#define SMB_MAX_NUM_SLAVE_BUFFER 2

/* Define this macro to receive more than 256 bytes during Slave Rx -
 * This feature uses DMA interrupts to clock stretch and reprogram the
 * Count register */
#define ENABLE_SLAVE_RX_MORE_THAN_256BYTES

/* Size of buffer on each controller (must be greater than 4) */
#define SLAVE1_BUFFER_SIZE  250 
#define SLAVE2_BUFFER_SIZE  64  
#define SLAVE3_BUFFER_SIZE  64  
#define SLAVE4_BUFFER_SIZE  80  //mctp layer buffer require 74 byte size in one packet
#define SLAVE5_BUFFER_SIZE  64  

/* alignment of slave buffers */
#define SMB_BB1_ALIGN 4
#define SMB_BB2_ALIGN 4
#define SMB_BB3_ALIGN 4
#define SMB_BB4_ALIGN 4
#define SMB_BB5_ALIGN 4

/* Maximum number of application per slave controller */
/* Glacier SMB porting */
#define SMB_MAX_NUM_SLAVE_APP 5
//----------------------------------------------------------------------
#define MEC_SMBUS0_BASE (SMB0_BASE_ADDRESS)
#define MEC_DMA_BASE (DMA_MAIN_BASE_ADDRESS)
#define MEC_DMA_CH0_BASE DMA_CHAN_BASE_ADDR(0)
#define MEC_DMA_CH1_BASE DMA_CHAN_BASE_ADDR(1)
#define MEC_DMA_CH2_BASE DMA_CHAN_BASE_ADDR(2)
#define MEC_DMA_CH3_BASE DMA_CHAN_BASE_ADDR(3)
#define MEC_DMA_CH4_BASE DMA_CHAN_BASE_ADDR(4)
#define MEC_DMA_CH5_BASE DMA_CHAN_BASE_ADDR(5)
#define MEC_DMA_CH6_BASE DMA_CHAN_BASE_ADDR(6)
#define MEC_DMA_CH7_BASE DMA_CHAN_BASE_ADDR(7)
#define MEC_DMA_CH8_BASE DMA_CHAN_BASE_ADDR(8)
#define MEC_DMA_CH9_BASE DMA_CHAN_BASE_ADDR(9)

#define MEC_SMBUS1_BASE (SMB1_BASE_ADDRESS)
#define MEC_SMBUS2_BASE (SMB2_BASE_ADDRESS)
#define MEC_SMBUS3_BASE (SMB3_BASE_ADDRESS)
#define MEC_SMBUS4_BASE (SMB4_BASE_ADDRESS)

#define HW_SMB_DEVICE_1 MEC_SMBUS0_BASE
#define HW_SMB_DEVICE_2 MEC_SMBUS1_BASE
#define HW_SMB_DEVICE_3 MEC_SMBUS2_BASE
#define HW_SMB_DEVICE_4 MEC_SMBUS3_BASE
#define HW_SMB_DEVICE_5 MEC_SMBUS4_BASE

#define HW_DMA_BLOCK_MAIN MEC_DMA_BASE
#define HW_DMA_BLOCK_1  MEC_DMA_CH0_BASE
#define HW_DMA_BLOCK_2  MEC_DMA_CH1_BASE
#define HW_DMA_BLOCK_3  MEC_DMA_CH2_BASE
#define HW_DMA_BLOCK_4  MEC_DMA_CH3_BASE
#define HW_DMA_BLOCK_5  MEC_DMA_CH4_BASE
#define HW_DMA_BLOCK_6  MEC_DMA_CH5_BASE
#define HW_DMA_BLOCK_7  MEC_DMA_CH6_BASE
#define HW_DMA_BLOCK_8  MEC_DMA_CH7_BASE
#define HW_DMA_BLOCK_9  MEC_DMA_CH8_BASE
#define HW_DMA_BLOCK_10 MEC_DMA_CH9_BASE

#define SMB_GIRQ  MCHP_SMB_GIRQ
#define DMA_GIRQ  MCHP_DMA_GIRQ

/* SMB Master operations use even numbered DMA channels */
#define SMB_MTR_DMA_BITPOS(smbnum) ((smbnum) * 2U)

/* SMB Slave operations use odd numbered DMA channels */
#define SMB_SLV_DMA_BITPOS(smbnum) (((smbnum) * 2U) + 1U)

/*Enable this to disable the master functionality*/
//#define DISABLE_SMB_MASTER

/*Enable this to handle power low to high event*/
//#define HANDLE_POWER_LOW_TO_HIGH_EVENT

#ifndef DISABLE_SMB_MASTER
#define SMB1_MASTER_DMA HW_DMA_BLOCK_2
#define SMB2_MASTER_DMA HW_DMA_BLOCK_4
#define SMB3_MASTER_DMA HW_DMA_BLOCK_6
#define SMB4_MASTER_DMA HW_DMA_BLOCK_8
#define SMB5_MASTER_DMA HW_DMA_BLOCK_10
#endif
#define SMB1_SLAVE_DMA  HW_DMA_BLOCK_1
#define SMB2_SLAVE_DMA  HW_DMA_BLOCK_3
#define SMB3_SLAVE_DMA  HW_DMA_BLOCK_5
#define SMB4_SLAVE_DMA  HW_DMA_BLOCK_7
#define SMB5_SLAVE_DMA  HW_DMA_BLOCK_9

//Update this macro as first master dma //dma irq bitmask
#define DMA_IRQ_SMB_MASTER_MASK (BIT_n_MASK(0))

#ifdef GET_SMB_GPIO_LOCATION /* This will be enabled only by smb_port.cpp */
typedef struct _smb_gpio_loc_
{
    UINT32 clk_gpio_control_addr;
    UINT32 input_gpio_addr_clk;
    UINT32 input_gpio_addr_data;
    UINT8 input_gpio_clk_bit;
    UINT8 input_gpio_data_bit;
}SMB_GPIO_LOCATION;

#if 0 /* Not needed */
uint32_t GPIO_INPUT_REG_ADDR[] = {
    (uint32_t) GPIO_PIN_REG_ADDRESS(0),
    (uint32_t) GPIO_PIN_REG_ADDRESS(1),
    (uint32_t) GPIO_PIN_REG_ADDRESS(2),
    (uint32_t) GPIO_PIN_REG_ADDRESS(3),
    (uint32_t) GPIO_PIN_REG_ADDRESS(4),
    (uint32_t) GPIO_PIN_REG_ADDRESS(5),
};
#endif

const SMB_GPIO_LOCATION smb_gpio[MAX_SMB][SMB_MAX_PORT_PER_CHANNEL] =
{
    /* Add entries here for expansion of channel & ports
     * There can be a case when number of ports can be different on different
     * controllers, in that case SMB_MAX_PORT_PER_CHANNEL will indicate the
     * maximum ports on any controller. In the below list provide an entry for invalid
     * ports by filling all values as 0, e.g. {0x0, 0x0, 0x0, 0x0, 0x0}*/

    /* All PORTS can be mapped to all the SMB controllers on the same GPIOs */
#if MAX_SMB > 0
    { /* CONTROLLER 1 */
        { GPIO_CTRL_REG_ADDRESS(GPIO_PIN_GPIO004), GPIO_PIN_REG_ADDRESS(0),
            GPIO_PIN_REG_ADDRESS(0), 4, 3 }, /* PORT0 */
        { 0x0, 0x0, 0x0, 0x0, 0x0 }, /* PORT1 */
        { 0x0, 0x0, 0x0, 0x0, 0x0 }, /* PORT2 */
        { 0x0, 0x0, 0x0, 0x0, 0x0 }, /* PORT3 */
        { GPIO_CTRL_REG_ADDRESS(GPIO_PIN_GPIO144), GPIO_PIN_REG_ADDRESS(3),
            GPIO_PIN_REG_ADDRESS(3), 4, 3 }, /* PORT4 */
        { 0x0, 0x0, 0x0, 0x0, 0x0 }, /* PORT5 */
        { GPIO_CTRL_REG_ADDRESS(GPIO_PIN_GPIO140), GPIO_PIN_REG_ADDRESS(3),
            GPIO_PIN_REG_ADDRESS(2), 0, 26 }, /* PORT6 */
        { 0x0, 0x0, 0x0, 0x0, 0x0 }, /* PORT7 */
        { 0x0, 0x0, 0x0, 0x0, 0x0 }, /* PORT8 */
        { GPIO_CTRL_REG_ADDRESS(GPIO_PIN_GPIO146), GPIO_PIN_REG_ADDRESS(3),
            GPIO_PIN_REG_ADDRESS(3), 6, 5 }, /* PORT9 */
        { GPIO_CTRL_REG_ADDRESS(GPIO_PIN_GPIO107), GPIO_PIN_REG_ADDRESS(2),
            GPIO_PIN_REG_ADDRESS(0), 7, 24 }, /* PORT10 */
        { 0x0, 0x0, 0x0, 0x0, 0x0 }, /* PORT11 */
        { 0x0, 0x0, 0x0, 0x0, 0x0 }, /* PORT12 */
        { 0x0, 0x0, 0x0, 0x0, 0x0 }, /* PORT13 */
        { 0x0, 0x0, 0x0, 0x0, 0x0 }, /* PORT14 */
        { GPIO_CTRL_REG_ADDRESS(GPIO_PIN_GPIO150), GPIO_PIN_REG_ADDRESS(2),
            GPIO_PIN_REG_ADDRESS(2), 8, 7 }, /* PORT15 */
    },
#endif /* #if MAX_SMB > 0 */
#if MAX_SMB > 1
    { /* CONTROLLER 2 */
        { GPIO_CTRL_REG_ADDRESS(GPIO_PIN_GPIO004), GPIO_PIN_REG_ADDRESS(0),
            GPIO_PIN_REG_ADDRESS(0), 4, 3 }, /* PORT0 */
        { 0x0, 0x0, 0x0, 0x0, 0x0 }, /* PORT1 */
        { 0x0, 0x0, 0x0, 0x0, 0x0 }, /* PORT2 */
        { 0x0, 0x0, 0x0, 0x0, 0x0 }, /* PORT3 */
        { GPIO_CTRL_REG_ADDRESS(GPIO_PIN_GPIO144), GPIO_PIN_REG_ADDRESS(3),
            GPIO_PIN_REG_ADDRESS(3), 4, 3 }, /* PORT4 */
        { 0x0, 0x0, 0x0, 0x0, 0x0 }, /* PORT5 */
        { GPIO_CTRL_REG_ADDRESS(GPIO_PIN_GPIO140), GPIO_PIN_REG_ADDRESS(3),
            GPIO_PIN_REG_ADDRESS(2), 0, 26 }, /* PORT6 */
        { 0x0, 0x0, 0x0, 0x0, 0x0 }, /* PORT7 */
        { 0x0, 0x0, 0x0, 0x0, 0x0 }, /* PORT8 */
        { GPIO_CTRL_REG_ADDRESS(GPIO_PIN_GPIO146), GPIO_PIN_REG_ADDRESS(3),
            GPIO_PIN_REG_ADDRESS(3), 6, 5 }, /* PORT9 */
        { GPIO_CTRL_REG_ADDRESS(GPIO_PIN_GPIO107), GPIO_PIN_REG_ADDRESS(2),
            GPIO_PIN_REG_ADDRESS(0), 7, 24 }, /* PORT10 */
        { 0x0, 0x0, 0x0, 0x0, 0x0 }, /* PORT11 */
        { 0x0, 0x0, 0x0, 0x0, 0x0 }, /* PORT12 */
        { 0x0, 0x0, 0x0, 0x0, 0x0 }, /* PORT13 */
        { 0x0, 0x0, 0x0, 0x0, 0x0 }, /* PORT14 */
        { GPIO_CTRL_REG_ADDRESS(GPIO_PIN_GPIO150), GPIO_PIN_REG_ADDRESS(2),
            GPIO_PIN_REG_ADDRESS(2), 8, 7 }, /* PORT15 */
    },
#endif /* #if MAX_SMB > 1 */
#if MAX_SMB > 2
    { /* CONTROLLER 3 */
        { GPIO_CTRL_REG_ADDRESS(GPIO_PIN_GPIO004), GPIO_PIN_REG_ADDRESS(0),
            GPIO_PIN_REG_ADDRESS(0), 4, 3 }, /* PORT0 */
        { 0x0, 0x0, 0x0, 0x0, 0x0 }, /* PORT1 */
        { 0x0, 0x0, 0x0, 0x0, 0x0 }, /* PORT2 */
        { 0x0, 0x0, 0x0, 0x0, 0x0 }, /* PORT3 */
        { GPIO_CTRL_REG_ADDRESS(GPIO_PIN_GPIO144), GPIO_PIN_REG_ADDRESS(3),
            GPIO_PIN_REG_ADDRESS(3), 4, 3 }, /* PORT4 */
        { 0x0, 0x0, 0x0, 0x0, 0x0 }, /* PORT5 */
        { GPIO_CTRL_REG_ADDRESS(GPIO_PIN_GPIO140), GPIO_PIN_REG_ADDRESS(3),
            GPIO_PIN_REG_ADDRESS(2), 0, 26 }, /* PORT6 */
        { 0x0, 0x0, 0x0, 0x0, 0x0 }, /* PORT7 */
        { 0x0, 0x0, 0x0, 0x0, 0x0 }, /* PORT8 */
        { GPIO_CTRL_REG_ADDRESS(GPIO_PIN_GPIO146), GPIO_PIN_REG_ADDRESS(3),
            GPIO_PIN_REG_ADDRESS(3), 6, 5 }, /* PORT9 */
        { GPIO_CTRL_REG_ADDRESS(GPIO_PIN_GPIO107), GPIO_PIN_REG_ADDRESS(2),
            GPIO_PIN_REG_ADDRESS(0), 7, 24 }, /* PORT10 */
        { 0x0, 0x0, 0x0, 0x0, 0x0 }, /* PORT11 */
        { 0x0, 0x0, 0x0, 0x0, 0x0 }, /* PORT12 */
        { 0x0, 0x0, 0x0, 0x0, 0x0 }, /* PORT13 */
        { 0x0, 0x0, 0x0, 0x0, 0x0 }, /* PORT14 */
        { GPIO_CTRL_REG_ADDRESS(GPIO_PIN_GPIO150), GPIO_PIN_REG_ADDRESS(2),
            GPIO_PIN_REG_ADDRESS(2), 8, 7 }, /* PORT15 */
    },
#endif /* #if MAX_SMB > 2 */
#if MAX_SMB > 3
    { /* CONTROLLER 4 */
        { GPIO_CTRL_REG_ADDRESS(GPIO_PIN_GPIO004), GPIO_PIN_REG_ADDRESS(0),
            GPIO_PIN_REG_ADDRESS(0), 4, 3 }, /* PORT0 */
        { 0x0, 0x0, 0x0, 0x0, 0x0 }, /* PORT1 */
        { 0x0, 0x0, 0x0, 0x0, 0x0 }, /* PORT2 */
        { 0x0, 0x0, 0x0, 0x0, 0x0 }, /* PORT3 */
        { GPIO_CTRL_REG_ADDRESS(GPIO_PIN_GPIO144), GPIO_PIN_REG_ADDRESS(3),
            GPIO_PIN_REG_ADDRESS(3), 4, 3 }, /* PORT4 */
        { 0x0, 0x0, 0x0, 0x0, 0x0 }, /* PORT5 */
        { GPIO_CTRL_REG_ADDRESS(GPIO_PIN_GPIO140), GPIO_PIN_REG_ADDRESS(3),
            GPIO_PIN_REG_ADDRESS(2), 0, 26 }, /* PORT6 */
        { 0x0, 0x0, 0x0, 0x0, 0x0 }, /* PORT7 */
        { 0x0, 0x0, 0x0, 0x0, 0x0 }, /* PORT8 */
        { GPIO_CTRL_REG_ADDRESS(GPIO_PIN_GPIO146), GPIO_PIN_REG_ADDRESS(3),
            GPIO_PIN_REG_ADDRESS(3), 6, 5 }, /* PORT9 */
        { GPIO_CTRL_REG_ADDRESS(GPIO_PIN_GPIO107), GPIO_PIN_REG_ADDRESS(2),
            GPIO_PIN_REG_ADDRESS(0), 7, 24 }, /* PORT10 */
        { 0x0, 0x0, 0x0, 0x0, 0x0 }, /* PORT11 */
        { 0x0, 0x0, 0x0, 0x0, 0x0 }, /* PORT12 */
        { 0x0, 0x0, 0x0, 0x0, 0x0 }, /* PORT13 */
        { 0x0, 0x0, 0x0, 0x0, 0x0 }, /* PORT14 */
        { GPIO_CTRL_REG_ADDRESS(GPIO_PIN_GPIO150), GPIO_PIN_REG_ADDRESS(2),
            GPIO_PIN_REG_ADDRESS(2), 8, 7 }, /* PORT15 */
    }
#endif /* #if MAX_SMB > 3 */
};
#endif /* #ifdef GET_SMB_GPIO_LOCATION */

/* Macro for smbus timer tick count in msec*/
#define SMBUS_TIMER_TICK_MSEC 10

#define TICK_PERIOD 10

/* Macros useful scheduling & calculating timing loops */
//#define ms_to_ticks(x)  (x/TICK_PERIOD)
//#define second_to_ticks(x)  (x*1000/TICK_PERIOD)

#ifndef DISABLE_PENDING_PACKETS
/** Packet expiry time*/ //for pending packets
#define SMB_PACKET_EXPIRE_TIME_TICKS ms_to_ticks(120) /* 120 ms */
#endif

// TIME INTERVALS USED DURING SMBUS POST PROCESSING
//----------------------------------------------------------------------
#define DETECT_CLK_DATA_HIGH_RETRY_COUNT 10
/* Hold Clock Low Timer Count - 40-50ms */
#define HOLD_CLK_LOW_TIMER_COUNT 5
/* Wait after bus reset Timer Count - 50-60ms */
#define WAIT_AFTER_BER_TIMER_COUNT 6
/* Time to check CLK & DATA high - 2000-2010ms (approx 2 seconds)*/
#define PERIOD_CHECK_CLK_DATA_HIGH 201
//----------------------------------------------------------------------

// SMBUS TIMING Values
//-----------------------------------------------------------------------------------------
//SMBus Baud Clock configuration

#define SMB_BAUD_CLK_8MHZ 0
#define SMB_BAUD_CLK_10MHZ 1
#define SMB_BAUD_CLK_16MHZ 2

//Define and enable the actual baud clock option as per hardware
#define SMB_BAUD_CLK SMB_BAUD_CLK_16MHZ
//#define SMB_BAUD_CLK SMB_BAUD_CLK_10MHZ
//#define SMB_BAUD_CLK SMB_BAUD_CLK_8MHZ


//-----------------------------------------------------------------------------------------
// SMBUS TIMING Values

#if (SMB_BAUD_CLK == SMB_BAUD_CLK_16MHZ)
    /* SMBus Timing values for 1MHz Speed */
    #define SPEED_1MHZ_BUS_CLOCK 0x0509
    #define SPEED_1MHZ_DATA_TIMING 0x06060601     //As per 16 MHz Baud Clock
    #define SPEED_1MHZ_DATA_TIMING_2 0x06
    #define SPEED_1MHZ_IDLE_SCALING 0x01000050 //As per 16 MHz Baud Clock
    #define SPEED_1MHZ_TIMEOUT_SCALING 0x149CC2C7

    //Add Values for 100Khz and 400Khz as per 16Mhz baud clock
    #define SPEED_400KHZ_BUS_CLOCK 0x0F17
    #define SPEED_400KHZ_DATA_TIMING 0x040A0A06     //As per 16 MHz Baud Clock
    #define SPEED_400KHZ_DATA_TIMING_2 0x0A
    #define SPEED_400KHZ_IDLE_SCALING 0x01000050 //As per 16 MHz Baud Clock
    #define SPEED_400KHZ_TIMEOUT_SCALING 0x159CC2C7

    #define SPEED_100KHZ_BUS_CLOCK 0x4F4F
    #define SPEED_100KHZ_DATA_TIMING 0x0C4D5006     //As per 16 MHz Baud Clock
    #define SPEED_100KHZ_DATA_TIMING_2 0x4D
    #define SPEED_100KHZ_IDLE_SCALING 0x01FC01ED //As per 16 MHz Baud Clock
    #define SPEED_100KHZ_TIMEOUT_SCALING 0x4B9CC2C7

#elif (SMB_BAUD_CLK == SMB_BAUD_CLK_10MHZ)

    #define SPEED_400KHZ_BUS_CLOCK 0x0A0D
    #define SPEED_400KHZ_DATA_TIMING 0x03080804     //As per 10 MHz Baud Clock
    #define SPEED_400KHZ_IDLE_SCALING 0x00A00032 //As per 10 MHz Baud Clock
    #define SPEED_400KHZ_TIMEOUT_SCALING 0x0A9DC4C8
    #define SPEED_400KHZ_DATA_TIMING_2 0x07

    #define SPEED_100KHZ_BUS_CLOCK 0x3131
    #define SPEED_100KHZ_DATA_TIMING 0x072A3104 //As per 10 MHz Baud Clock
    #define SPEED_100KHZ_IDLE_SCALING 0x01400136 //As per 10 MHz Baud Clock
    #define SPEED_100KHZ_TIMEOUT_SCALING 0x30C6F5FB
    #define SPEED_100KHZ_DATA_TIMING_2 0x2B

#else //(SMB_BAUD_CLK == SMB_BAUD_CLK_8MHZ)
    /* For 8MHz baud clock, set the high & low period to 10,
     * therefore 8M/(10+10) = 400khz */
    #define SPEED_400KHZ_BUS_CLOCK 0x080a
    #define SPEED_400KHZ_DATA_TIMING 0x02060603  //As per 8 MHz Baud Clock
    /* T_Idle_Delay = 17us, T_Idle_Window = 6us */
    #define SPEED_400KHZ_IDLE_SCALING 0x0084002A  //As per 8 MHz Baud Clock
    #define SPEED_400KHZ_TIMEOUT_SCALING 0x0A9DC4C8
    #define SPEED_400KHZ_DATA_TIMING_2 0x05

    /* For 8MHz baud clock, set the high & low period to 40,
     * therefore 8M/(40+40) = 100khz */
    #define SPEED_100KHZ_BUS_CLOCK 0x2727
    #define SPEED_100KHZ_DATA_TIMING 0x05222703 //As per 8 MHz Baud Clock
    /* T_Idle_Delay = 32us, T_Idle_Window = 31us */
    #define SPEED_100KHZ_IDLE_SCALING 0x010000F8  //As per 8 MHz Baud Clock

    /* Bus Idle Min(Byte3 of Time-Out Scaling Register) = 4.7us
     * Master Cum Time Out(Byte2 of Time-Out Scaling Register) = 10ms
     * Slave Cum Time Out(Byte1 of Time-Out Scaling Register) = 25ms
     * Clock High Time Out(Byte 0 of Time-Out Scaling Register) = 50us
     */
    #define SPEED_100KHZ_TIMEOUT_SCALING 0x269DC4C8
#endif
//-----------------------------------------------------------------------------------------

size_t popcount(uintmax_t num);
#define PRECISION(umax_value) popcount(umax_value)

#endif /* SMB_CFG_CEC173X_H_ */
/** @}
*/
