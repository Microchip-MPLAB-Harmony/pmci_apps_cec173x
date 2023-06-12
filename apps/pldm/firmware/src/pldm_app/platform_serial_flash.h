/***************************************************************************** 
* Copyright 2018 Microchip Technology Inc. and its subsidiaries.                       
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
                                                                               
/** @file platform_serial_flash.h
 * flash_program 
 */
/** @defgroup flash_program 
 */
 
#ifndef PLATFORM_SERIAL_FLASH_H                                                          
#define PLATFORM_SERIAL_FLASH_H 

typedef struct {    
    uint16_t write_len;    
    uint8_t read_len;
    uint8_t rsvd1;
    uint8_t rsvd2;
    uint8_t *write_buf;
    uint8_t *read_buf;    
} SERIAL_FLASH_COMMAND;

void platform_qmspi_init(uint8_t spi_port);
uint8_t platform_serial_flash_check_done_status(uint32_t *qmspi_status);
uint8_t platform_serial_flash_process_cmd (SERIAL_FLASH_COMMAND *cmd, uint32_t *qmspi_status);
uint8_t platform_serial_flash_read_dma(uint32_t spi_addr, uint32_t mem_addr, uint32_t data_len, uint8_t mode, uint32_t *qmspi_status);
uint8_t platform_serial_flash_write_page(uint32_t spi_addr,uint32_t mem_addr, uint32_t data_len, uint32_t *qmspi_status);

void chipSelectSPI(uint8_t state);

void spi_port_sel_set(uint8_t spi_port);

void component_set(uint8_t comp);

/******************************************************************************/
/** release_qmspi_port();
* Release the qmspi port after usage
* @param port - required spi port to be released
* @param io_mode - SPI IO Modes Single, dual or quad
* @return none
*******************************************************************************/
void release_qmspi_port(uint8_t port, uint8_t io_mode);

void sb_spi_tristate();

void spi_init_int_spi();

void spi_init_shd_spi();

/** CHIP SELECT */
typedef enum {
    SELECT,
    DESELECT
}CS_SPI;

#endif                                                                         
/* end platform_serial_flash.h */                                                         
/**   @}                                                                       
 */ 
