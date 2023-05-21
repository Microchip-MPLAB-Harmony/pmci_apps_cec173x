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
                                                                               
/** @file app_qmspi.h                                                         
 * app_qmspi                                                           
 */                                                                            
/** @defgroup app_qmspi                                               
 */                                                                            
#ifndef _APP_QMSPI_H                                                          
#define _APP_QMSPI_H                                                          

#include "common.h"
/** Define the device either one to test MEC2016 or Everglades or Carlsbad */
//#define EVERGLADES
//#define MEC2016
#define CARLSBAD

 /******************************************************************************/
/** Local function Declarations*/ 
void app_qmspi_main(void);
void app_qmspi_init(void);
void app_qmspi_read(void);
void app_qmspi_read_dma(uint32_t spi_addr,uint32_t mem_addr,uint32_t data_len, uint8_t mode);
void app_qmspi_page_erase(uint32_t spi_addr);
void app_qmspi_write(uint32_t spi_addr,uint32_t mem_addr,uint32_t data_len);
typedef uint32_t (*FP_RU32_V)(void);
/******************************************************************************/ 

/**API Function Prototypes */
uint8_t api_qmspi_flash_cmd(uint32_t ntx, uint8_t *ptx, uint32_t nrx, uint8_t *prx, FP_RU32_V ftmout);
typedef void (*api_qmspi_init_td)(uint8_t spi_mode, uint8_t freq_div, uint8_t ifc , uint8_t port);
typedef uint8_t (*api_qmspi_port_ctrl_td)(uint8_t port_id, uint8_t width_mask, uint8_t enable);
typedef uint8_t (*api_qmspi_flash_cmd_td)(uint32_t ntx, uint8_t *ptx, uint32_t nrx, uint8_t *prx, FP_RU32_V ftmout, uint8_t port);
typedef uint8_t (*api_qmspi_is_done_status_td)(uint32_t *pstatus, uint8_t port);
typedef uint8_t (*api_qmspi_flash_program_dma_td)(uint8_t prog_cmd, uint32_t spi_addr, uint32_t nbytes, uint8_t port);
typedef uint8_t (*api_dma_dev_xfr_cfg_td)(uint8_t chan_id, uint32_t dev_id, uint32_t maddr, uint32_t nbytes, uint32_t flags);
typedef void (*api_qmspi_start_td)(uint16_t ien_mask,uint8_t port );
typedef uint32_t (*api_qmspi_flash_read24_dma_td)(uint8_t cmd_id, uint32_t spi_addr, uint32_t nbytes, uint32_t maddr,uint8_t port);
typedef uint8_t (*api_qmspi_port_drv_slew_td)(uint8_t port_id, uint8_t width_mask, uint8_t drv_slew);
typedef uint32_t(*api_qmspi_flash_read_24_32_dma_td)(uint8_t cmd_id, uint32_t spi_addr, uint32_t nbytes, uint32_t maddr, uint8_t port);
typedef uint32_t(*api_qmspi_flash_read_24_32_ldma_td)(uint8_t cmd_id, uint32_t spi_addr, uint32_t nbytes, uint32_t maddr, uint8_t port);

/******************************************************************************/
#define api_qmspi_init                        ((api_qmspi_init_td) 0x0000f225)
#define api_qmspi_port_ctrl                   ((api_qmspi_port_ctrl_td)    0x0000f215)
#define api_qmspi_flash_cmd                     ((api_qmspi_flash_cmd_td) 0x0000f235)
#define api_qmspi_is_done_status                ((api_qmspi_is_done_status_td)0x0000f221)
#define api_qmspi_flash_program_dma              ((api_qmspi_flash_program_dma_td)0x0000f239)
#define api_dma_dev_xfr_cfg                      ((api_dma_dev_xfr_cfg_td)  0x0000f211)
#define api_qmspi_start                          ((api_qmspi_start_td) 0x0000f231)
#define api_qmspi_flash_read24_dma               ((api_qmspi_flash_read24_dma_td) 0x0000f22d)
#define api_qmspi_port_drv_slew                   ((api_qmspi_port_drv_slew_td) 0x0000f219)
#define api_qmspi_flash_read_24_32_dma            ((api_qmspi_flash_read_24_32_dma_td)0x0000f23d)
#define api_qmspi_flash_read_24_32_ldma            ((api_qmspi_flash_read_24_32_ldma_td)0x0000f241)

    #define DMA_CH00_ID (0u)
    #define DMA_CH01_ID (1u)
    #define DMA_CH02_ID (2u)
    #define DMA_CH03_ID (3u)
    #define DMA_CH04_ID (4u)
    #define DMA_CH05_ID (5u)
    #define DMA_CH06_ID (6u)
    #define DMA_CH07_ID (7u)
    #define DMA_CH08_ID (8u)
    #define DMA_CH09_ID (9u)
    #define DMA_CH10_ID (10u)
    #define DMA_CH11_ID (11u)
    #define DMA_CH12_ID (12u)
    #define DMA_CH13_ID (13u)
    #define DMA_MAX_ID  (14u)

    // DMA Device ID in bits[7:1] and DMA direction in bit[0]
    // Matches DMA Control bits[15:8]
    #define QMSPI_TX_DMA_REQ_DIR0   ((DMA_CH10_ID << 1) + (1ul)) // Transmit is Memory to Device
    #define QMSPI_RX_DMA_REQ_DIR0   ((DMA_CH11_ID << 1) + (0ul)) // Receive is Device to Memory

    #define QMSPI1_TX_DMA_REQ_DIR0   ((DMA_CH12_ID << 1) + (1ul)) // Transmit is Memory to Device
    #define QMSPI1_RX_DMA_REQ_DIR0   ((DMA_CH13_ID << 1) + (0ul)) // Receive is Device to Memory

    #define DMA_DEVICE_ID   QMSPI_RX_DMA_REQ_DIR0
    #define DMA_CHANNEL_ID  DMA_CH07_ID //Config any DMA Channel for the opeartion

/******************************************************************************/
 
 
/******************************************************************************/
/** QMSPI Device common macros  */ 
#define DEV_ENABLE   1u
#define DEV_DISABLE  0u
#define NONE     0u 
 
/** QMSPI Supported interfaces */
#define SHD_SPI         0u   /*<QMSPI Port 0 = Shared SPI signals */
#define PVT_SPI         1u   /*<QMSPI Port 1 = Private SPI signals */
#define INT_SPI         2u   /*<QMSPI Port 2 = Internal SPI signals */
 
 /** QMSPI PIN Width Mask*/
#define SPI_IO_FD_DUAL  0x0Eu
#define SPI_IO_QUAD     0x3Eu


/** QMSPI SPI Mode selection 
 * Used when accessing MODE register 
 * SPI signalling mode field.
 */
typedef enum
{
    QMSPI_SPI_MODE_0 = 0x00u,
    QMSPI_SPI_MODE_1 = 0x06u,
    QMSPI_SPI_MODE_2 = 0x01u,
    QMSPI_SPI_MODE_3 = 0x07u
} QMSPI_SPI_MODE;

/** SPI Flash to Use to read the image */
typedef enum
{
    SPI_SELECT_SHARED_COMP_0 = 0x00, //00_00
    SPI_SELECT_SHARED_COMP_1, //00_01
    SPI_SELECT_RESERVED1,
    SPI_SELECT_RESERVED2,
    SPI_SELECT_PVT_COMP_0,    //01_00
    SPI_SELECT_PVT_COMP_1,    //01_01
    SPI_SELECT_RESERVED3,
    SPI_SELECT_RESERVED4,
    SPI_SELECT_INT_COMP_0,    //10_00
    SPI_SELECT_MAX
} SPI_FLASH_SELECT;

/** QMSPI SPI Read Operations
 */
typedef enum
{
    QMSPI_SPI_SINGLE      = 0u,
    QMSPI_SPI_FAST_SINGLE = 1u,
    QMSPI_SPI_FAST_DUAL   = 2u,
    QMSPI_SPI_FAST_QUAD   = 3u
} QMSPI_SPI_READ;


/** QMSPI SPI frequency selection  */
typedef enum
{
    QMSPI_FREQ_96M = 1u,
	QMSPI_FREQ_48M = 2u,
    QMSPI_FREQ_24M = 4u,
    QMSPI_FREQ_16M = 6u,
    QMSPI_FREQ_12M = 8u,
    QMSPI_FREQ_08M = 12u,
    QMSPI_FREQ_06M = 16u,
    QMSPI_FREQ_04M = 24u,
    QMSPI_FREQ_03M = 32u,
    QMSPI_FREQ_02M = 48u,
    QMSPI_FREQ_01M = 96u
} QMSPI_FREQ;
/******************************************************************************/

#endif                                                                         
/* end app_qmspi.h */                                                         
/**   @}                                                                       
 */                                                                            
