/***************************************************************************** 
* Copyright 2020 Microchip Technology Inc. and its subsidiaries.                       
* You may use this software and any derivatives exclusively with               
* Microchip products.                                                          
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP 'AS IS'.                              
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
                                                                               
/** @file sb_gpio.h
 * secure boot - GPIO
 */
/** @defgroup secure_boot 
 */
 
#ifndef SB_GPIO_H                                                          
#define SB_GPIO_H

#include "common.h"

#ifdef TIMING_MEASUREMENT
#define UNUSED_GPIOS_COUNT      3
#else
#define UNUSED_GPIOS_COUNT      4
#endif


/* APx Reset Pins */
#define PIN_AP0_RESET              GPIO_PIN_GPIO106
#define PIN_AP1_RESET              GPIO_PIN_GPIO131
#define PIN_EXTRST                 GPIO_PIN_GPIO012
#define PIN_AP0_RESET_ALT          GPIO_PIN_GPIO107
#define PIN_AP1_RESET_ALT          GPIO_PIN_GPIO112

/* SPI Bleed pins */
#define PIN_SPI0_BLEED_SEL          GPIO_PIN_GPIO027
#define PIN_SPI1_BLEED_SEL          GPIO_PIN_GPIO033
    
/* Fatal Error PIN */
#define PIN_FATAL_ERROR             GPIO_PIN_GPIO003

/* Remote Acces Pin */
#define PIN_REMOTE_ACCESS           GPIO_PIN_GPIO144

/* Reset Monitoring */
#define PIN_WDTRST2                 GPIO_PIN_GPIO013
#define PIN_EXTRST_IN               GPIO_PIN_GPIO063
#define PIN_ASYNC_RST_DET           GPIO_PIN_GPIO050
#define PIN_AP0_HBLED               GPIO_PIN_GPIO015
#define PIN_AP1_RESET_IN            GPIO_PIN_GPIO113

#define PIN_TEST_BYPASS             GPIO_PIN_GPIO123
#define PIN_PLTRST                  GPIO_PIN_GPIO047

/* LED Pins */
/* LED Support */
#define PIN_LED0                    GPIO_PIN_GPIO156
#define PIN_LED1                    GPIO_PIN_GPIO157
    
/* EC Status LED */
#define PIN_EC_STS                  GPIO_PIN_GPIO053

    #define PIN_EC_HEALTH           GPIO_PIN_GPIO024

#define PIN_TIME_MEASUREMENT        GPIO_PIN_GPIO031
/* QSPI0 and QSPI1 OUTPUT PINS*/

#define PIN_QSPI0_CS0               GPIO_PIN_GPIO055
#define PIN_QSPI0_CS1               GPIO_PIN_GPIO002
#define PIN_QSPI0_CLK               GPIO_PIN_GPIO056
#define PIN_QSPI0_IO0               GPIO_PIN_GPIO223
#define PIN_QSPI0_IO1               GPIO_PIN_GPIO224
#define PIN_QSPI0_IO2               GPIO_PIN_GPIO227
#define PIN_QSPI0_IO3               GPIO_PIN_GPIO016

#define PIN_QSPI1_CS0               GPIO_PIN_GPIO124
#define PIN_QSPI1_CS1               GPIO_PIN_GPIO120
#define PIN_QSPI1_CLK               GPIO_PIN_GPIO125
#define PIN_QSPI1_IO0               GPIO_PIN_GPIO121
#define PIN_QSPI1_IO1               GPIO_PIN_GPIO122
#define PIN_QSPI1_IO2               GPIO_PIN_GPIO123
#define PIN_QSPI1_IO3               GPIO_PIN_GPIO126

/* Internal Flash PIN */
#define PIN_INT_QSPI0_CS            GPIO_PIN_GPIO230
#define PIN_INT_QSPI0_CLK           GPIO_PIN_GPIO231
#define PIN_INT_QSPI0_IO0           GPIO_PIN_GPIO232
#define PIN_INT_QSPI0_IO1           GPIO_PIN_GPIO233
#define PIN_INT_QSPI0_IO2           GPIO_PIN_GPIO234
#define PIN_INT_QSPI0_IO3           GPIO_PIN_GPIO235

/* QSPI0 and QSPI1 INPUT PINS*/

#define PIN_QSPI0_IN_CS0            GPIO_PIN_GPIO020
#define PIN_QSPI0_IN_CS1            GPIO_PIN_GPIO021
#define PIN_QSPI0_IN_CLK            GPIO_PIN_GPIO204
#define PIN_QSPI0_IN_IO0            GPIO_PIN_GPIO023
#define PIN_QSPI0_IN_IO1            GPIO_PIN_GPIO022
#define PIN_QSPI0_IN_IO2            GPIO_PIN_GPIO202
#define PIN_QSPI0_IN_IO3            GPIO_PIN_GPIO203

#define PIN_QSPI1_IN_CS0            GPIO_PIN_GPIO071
#define PIN_QSPI1_IN_CS1            GPIO_PIN_GPIO045
#define PIN_QSPI1_IN_CLK            GPIO_PIN_GPIO200
#define PIN_QSPI1_IN_IO0            GPIO_PIN_GPIO070
#define PIN_QSPI1_IN_IO1            GPIO_PIN_GPIO032
#define PIN_QSPI1_IN_IO2            GPIO_PIN_GPIO165
#define PIN_QSPI1_IN_IO3            GPIO_PIN_GPIO171


#define I2C00_SDA_GPIO003           GPIO_PIN_GPIO003
#define I2C00_SCL_GPIO004           GPIO_PIN_GPIO004
                               
#define I2C04_SDA_GPIO143           GPIO_PIN_GPIO143
#define I2C04_SCL_GPIO144           GPIO_PIN_GPIO144
                                
#define I2C06_SDA_GPIO132           GPIO_PIN_GPIO132
#define I2C06_SCL_GPIO140           GPIO_PIN_GPIO140
                                
#define I2C09_SDA_GPIO145           GPIO_PIN_GPIO145
#define I2C09_SCL_GPIO146           GPIO_PIN_GPIO146
                                
#define I2C10_SDA_GPIO030           GPIO_PIN_GPIO030
#define I2C10_SCL_GPIO107           GPIO_PIN_GPIO107
                               
#define I2C15_SDA_GPIO147           GPIO_PIN_GPIO147
#define I2C15_SCL_GPIO150           GPIO_PIN_GPIO150

/* ISPI IO Pins */
#define PIN_ISPI_CS                 GPIO_PIN_GPIO241
#define PIN_ISPI_CLK                GPIO_PIN_GPIO242
#define PIN_ISPI_IO0                GPIO_PIN_GPIO243
#define PIN_ISPI_IO1                GPIO_PIN_GPIO244
#define PIN_ISPI_IO2                GPIO_PIN_GPIO245 
#define PIN_ISPI_IO3                GPIO_PIN_GPIO246 
#define PIN_TST_CLK_OUT             GPIO_PIN_GPIO253
#define INT_NRESET_EMC              GPIO_PIN_GPIO252
#define PIN_INT_32KHZ               GPIO_PIN_GPIO254
#define PIN_INT_EMC_INTR            GPIO_PIN_GPIO255

#define PIN_SPI0_KILL               GPIO_PIN_GPIO000
#define PIN_SPI1_KILL               GPIO_PIN_GPIO163

#define I2C_MAX_PORT                (6)

#define QSPI_PIN_MAX    0x07
#define QSPI_PIN_CFG_MAX    0x05

#define MPU_gpio_init_intdet_edge_fall_pu(pin_x)    gpio_init( pin_x, GPIO_INP_ENABLE, GPIO_FUNCTION_GPIO, GPIO_POLARITY_NON_INVERTED, GPIO_DIR_INPUT, \
                                                   GPIO_OUTPUT_BUFFER_TYPE_OPEN_DRAIN, GPIO_INTDET_TYPE_FALLING_EDGE, GPIO_PWR_VTR, GPIO_PULL_TYPE_UP );
    
#define MPU_gpio_init_intdet_edge_fall_pd(pin_x)    gpio_init( pin_x, GPIO_INP_ENABLE, GPIO_FUNCTION_GPIO, GPIO_POLARITY_NON_INVERTED, GPIO_DIR_INPUT, \
                                                   GPIO_OUTPUT_BUFFER_TYPE_OPEN_DRAIN, GPIO_INTDET_TYPE_FALLING_EDGE, GPIO_PWR_VTR, GPIO_PULL_TYPE_DOWN );
    
#define MPU_gpio_init_intdet_edge_rise_pd(pin_x)    gpio_init( pin_x, GPIO_INP_ENABLE, GPIO_FUNCTION_GPIO, GPIO_POLARITY_NON_INVERTED, GPIO_DIR_INPUT, \
                                                   GPIO_OUTPUT_BUFFER_TYPE_OPEN_DRAIN, GPIO_INTDET_TYPE_RISING_EDGE, GPIO_PWR_VTR, GPIO_PULL_TYPE_DOWN );
    
#define MPU_gpio_init_intdet_edge_both_pd(pin_x)    gpio_init( pin_x, GPIO_INP_ENABLE, GPIO_FUNCTION_GPIO, GPIO_POLARITY_NON_INVERTED, GPIO_DIR_INPUT, \
                                                   GPIO_OUTPUT_BUFFER_TYPE_OPEN_DRAIN, GPIO_INTDET_TYPE_EITHER_EDGE, GPIO_PWR_VTR, GPIO_PULL_TYPE_DOWN );

/******************************************************************************/
/** sb_gpio_disable_unused();
* Disable routine for unused gpios
* @param void - None
* @return void - None
*******************************************************************************/
void sb_gpio_disable_unused(void);

/******************************************************************************/
/** gpio_init_od_input();
* Sets GPIO to open drain input
* @param pin_id GPIO pin id
* @return None
*******************************************************************************/
void gpio_init_od_input(GPIO_PIN pin_id);

/******************************************************************************/
/** gpio_init_pp_output();
* Sets GPIO to push pull output 
* @param pin_id GPIO pin id
* @return None
*******************************************************************************/
void gpio_init_pp_output(GPIO_PIN pin_id);

/******************************************************************************/
/** gpio_init_pp_input();
* Sets GPIO to push pull input configuration
* @param pin_id GPIO pin id
* @return None
*******************************************************************************/
void gpio_init_pp_input(GPIO_PIN pin_id);

/******************************************************************************/
/** gpio_init_pp_input_ISR();
* Sets GPIO to push pull input configuration from ISR
* @param pin_id GPIO pin id
* @return None
*******************************************************************************/
void gpio_init_pp_input_ISR(GPIO_PIN pin_id);

/******************************************************************************/
/** gpio_init_pp_output_low();
* Sets GPIO to push pull output low
* @param pin_id GPIO pin id
* @return None
*******************************************************************************/
void gpio_init_pp_output_low(GPIO_PIN pin_id);

/******************************************************************************/
/** gpio_init_pp_output_low_ISR();
* Sets GPIO to push pull output low from ISR
* @param pin_id GPIO pin id
* @return None
*******************************************************************************/
void gpio_init_pp_output_low_ISR(GPIO_PIN pin_id);

/******************************************************************************/
/** gpio_init_pp_output_high();
* Sets GPIO to push pull output high
* @param pin_id GPIO pin id
* @return None
*******************************************************************************/
void gpio_init_pp_output_high(GPIO_PIN pin_id);

/******************************************************************************/
/** sb_gpio_init_required_pins 
*  Initializes critical and REQ pins present in all platforms
* @param None
* @return None
*******************************************************************************/
void sb_gpio_init_required_pins(void);

/******************************************************************************/
/** sb_gpio_init_pins_based_on_otp 
*  Initializes pins based on OTP settings common to all
* @param None
* @return None
*******************************************************************************/
void sb_gpio_init_pins_based_on_otp(void);

/******************************************************************************/
/** sb_gpio_EXTRST_IN_low_transition_check 
*  Checks if EXTRST_IN has transitioned to low
* @param None
* @return 1 if EXTRST_IN has transitioned low
*******************************************************************************/
uint8_t sb_gpio_EXTRST_IN_low_transition_check(void);

/******************************************************************************/
/** sb_gpio_ASYNC_RST_DET_low_transition_check 
*  Checks if ASYNC_RST_DET has transitioned to low
* @param None
* @return 1 if ASYNC_RST_DET has transitioned low
*******************************************************************************/
uint8_t sb_gpio_ASYNC_RST_DET_low_transition_check(void);


/******************************************************************************/
/** sb_gpio_qspi_in_list()
*  Return the address of requested port input pin list
* @param channel
* @return pin_list_address
*******************************************************************************/
GPIO_PIN* sb_gpio_qspi_in_list(uint8_t channel);

/******************************************************************************/
/** sb_gpio_qspi_out_list()
*  Return the address of requested port output pin list
* @param channel
* @return pin_list_address
*******************************************************************************/
GPIO_PIN* sb_gpio_qspi_out_list(uint8_t channel);

/******************************************************************************/
/** sb_gpio_qspi_cs_get()
*  Return the QSPI Chip select pin based on spi_select and qspi_dir
* @param spi_select
* @param qspi_dir - Input / Output
* @return cs_pin
*******************************************************************************/
GPIO_PIN sb_gpio_qspi_cs_get(uint8_t spi_select, uint8_t qspi_dir);

/******************************************************************************/
/** sb_gpio_kill_get()
*  Return the SPI KILL pin based on SPIMON Channel
* @param mon_channel - Refer enum SB_SPIMON_CHANNEL
* @return SPI Kill pin
*******************************************************************************/
GPIO_PIN sb_gpio_kill_get(uint8_t mon_channel);

/******************************************************************************/
/** sb_gpio_init_pins_table();
* Init GPIO pin tables
* @param void - None
* @return void - None
*******************************************************************************/
void sb_gpio_init_pins_table(void);

#endif                                                                         
/* end SB_GPIO_H */                                                         
/**   @}                                                                       
 */ 


