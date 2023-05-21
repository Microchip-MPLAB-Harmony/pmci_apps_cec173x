/*****************************************************************************
* � 2017 Microchip Technology Inc. and its subsidiaries.
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
#include <stdio.h>  
#include <stdlib.h>
#include <string.h>
#include "rom_qmspi.h"
#include "platform_serial_flash.h"
#include "sb_gpio.h"
#include "common.h"

uint8_t spi_port_sel;
uint8_t internal_flash;
uint8_t component_sel;
/******************************************************************************/

/** spi_port_sel_set();
 * 
 * @param spi_port - internal or shared or private flash
 *        spi_port_sel - spi port selected for erase/read/write operations
 * @return None
 *******************************************************************************/
void spi_port_sel_set(uint8_t spi_port)
{
    if (spi_port == INT_SPI){
        spi_port_sel = 0;
        internal_flash = 1;
    } else if (spi_port == SHD_SPI) {
        spi_port_sel = SHD_SPI;
        internal_flash = 0;
    } else if (spi_port == PVT_SPI) {
        spi_port_sel = PVT_SPI;
        internal_flash = 0;
    }

}

void component_set(uint8_t comp)
{
    if (comp == 0){
        component_sel = 0;
    } else if (comp = 1) {
        component_sel = 1;
    }
}

void chipSelectSPI(uint8_t state)
{
    if (internal_flash == 1) {
        gpio_output_set(PIN_INT_QSPI0_CS, GPIO_ALT_OUT_DIS, (uint32_t)(state) );
    } else {
        if (spi_port_sel == SHD_SPI && component_sel == 0){
            gpio_output_set( PIN_QSPI0_CS0, GPIO_ALT_OUT_DIS, (uint32_t)(state) );
        } else if (spi_port_sel == SHD_SPI && component_sel == 1) {
            gpio_output_set( PIN_QSPI0_CS1, GPIO_ALT_OUT_DIS, (uint32_t)(state) );
        } else if (spi_port_sel == PVT_SPI && component_sel == 0) {
            gpio_output_set( PIN_QSPI1_CS0, GPIO_ALT_OUT_DIS, (uint32_t)(state) );
        } else if (spi_port_sel == PVT_SPI && component_sel == 1) {
            gpio_output_set( PIN_QSPI1_CS1, GPIO_ALT_OUT_DIS, (uint32_t)(state) );
        }
    }

}

//Retry count for polling 'api_qmspi_is_done_status' in milliseconds
#define QMSPI_STATUS_RETRY_COUNT   200


/******************************************************************************/
/** release_qmspi_port();
* Release the qmspi port after usage
* @param port - required spi port to be released
* @param io_mode - SPI IO Modes Single, dual or quad
* @return none
*******************************************************************************/
void release_qmspi_port(uint8_t port, uint8_t io_mode)
{
    api_qmspi_port_ctrl(port, io_mode, DEV_DISABLE);

    if(port==SHD_SPI)
    {
        /* soft reset the qmspi port */
        QMSPI0_REGS->QMSPI_MODE = 0x02;
        QMSPI0_REGS->QMSPI_BUF_CNT_STS = QMSPI0_REGS->QMSPI_MODE;
        interrupt_device_ecia_source_clear(ECIA_AGG_INT_SRC_QMSPI0);

        gpio_property_set(PIN_QSPI0_CS0, GPIO_PROP_ALL, 0x40 );
        gpio_property_set(PIN_QSPI0_CS1, GPIO_PROP_ALL, 0x40 );

    }

    else if(port==PVT_SPI)
    {
        /* soft reset the qmspi port */
        QMSPI1_REGS->QMSPI_MODE = 0x02;
        QMSPI1_REGS->QMSPI_BUF_CNT_STS = QMSPI1_REGS->QMSPI_MODE;
        interrupt_device_ecia_source_clear(ECIA_AGG_INT_SRC_QMSPI1);

        gpio_property_set(PIN_QSPI1_CS1, GPIO_PROP_ALL, 0x40 );
        gpio_property_set(PIN_QSPI1_CS1, GPIO_PROP_ALL, 0x40 );

    }

    else if(port == INT_SPI)
    {
        /* soft reset the qmspi port */
        QMSPI0_REGS->QMSPI_MODE = 0x02;
        QMSPI0_REGS->QMSPI_BUF_CNT_STS = QMSPI0_REGS->QMSPI_MODE;

        interrupt_device_ecia_source_clear(ECIA_AGG_INT_SRC_QMSPI0);

        gpio_property_set(PIN_INT_QSPI0_CS, GPIO_PROP_ALL, 0x40 );

    }
}

void sb_spi_tristate()
{

    release_qmspi_port(INT_SPI, SPI_IO_FD_DUAL);
    timer_delay_us(20);
    release_qmspi_port(SHD_SPI, SPI_IO_FD_DUAL);
    timer_delay_us(20);
    
}

void spi_init_int_spi()
{
  api_qmspi_port_ctrl(SHD_SPI, SPI_IO_FD_DUAL, DEV_DISABLE);
  api_qmspi_port_ctrl(PVT_SPI, SPI_IO_FD_DUAL, DEV_DISABLE);
  api_qmspi_port_ctrl(INT_SPI, SPI_IO_FD_DUAL, DEV_ENABLE);
  api_qmspi_port_drv_slew(INT_SPI, SPI_IO_FD_DUAL, (GPIO_DRV_4MA) | (GPIO_SLEW_FAST));
  api_qmspi_init(QMSPI_SPI_MODE_0, QMSPI_FREQ_24M, NONE, SHD_SPI);
  gpio_init_pp_output_high(PIN_QSPI0_CS0);
  gpio_init_pp_output_high(PIN_QSPI0_CS1);
  gpio_init_pp_output_high(PIN_QSPI1_CS0);
  gpio_init_pp_output_high(PIN_QSPI1_CS1);
  gpio_init_pp_output_high(PIN_INT_QSPI0_CS);

}

void spi_init_shd_spi()
{
  api_qmspi_port_ctrl(INT_SPI, SPI_IO_FD_DUAL, DEV_DISABLE);
  api_qmspi_port_ctrl(PVT_SPI, SPI_IO_FD_DUAL, DEV_DISABLE);
  api_qmspi_port_ctrl(SHD_SPI, SPI_IO_FD_DUAL, DEV_ENABLE);
  api_qmspi_port_drv_slew(SHD_SPI, SPI_IO_FD_DUAL, (GPIO_DRV_4MA) | (GPIO_SLEW_FAST));
  api_qmspi_init(QMSPI_SPI_MODE_0, QMSPI_FREQ_24M, NONE, SHD_SPI);
  gpio_init_pp_output_high(PIN_QSPI0_CS0);
  gpio_init_pp_output_high(PIN_QSPI0_CS1);
  gpio_init_pp_output_high(PIN_QSPI1_CS0);
  gpio_init_pp_output_high(PIN_QSPI1_CS1);
  gpio_init_pp_output_high(PIN_INT_QSPI0_CS);

}

/******************************************************************************/
/** platform_qmspi_init();
* intialise the SPI interface for the port selection , frequency, mode and port 
* type of access for the pin ocnfiguration.
* Supported interface are Shared SPI, Private SPI and internal SPI
* Supported read\write mode are Single Dual and QUAD
* @param spi_port
* @return void
*******************************************************************************/
void platform_qmspi_init(uint8_t spi_port){
    
    api_qmspi_port_ctrl(spi_port, SPI_IO_FD_DUAL, DEV_ENABLE);

    api_qmspi_init(QMSPI_SPI_MODE_0, QMSPI_FREQ_24M, NONE, spi_port_sel);
    
    DMA_MAIN_REGS->DMA_MAIN_ACTRST = 1;
}

uint8_t platform_serial_flash_check_done_status(uint32_t *qmspi_status)
{
    uint16_t count;
    uint8_t ret_val, done_status;
    
    count = QMSPI_STATUS_RETRY_COUNT;    
    do 
    {
        done_status = api_qmspi_is_done_status(qmspi_status, spi_port_sel);
        timer_delay_ms(1);
        count--;
        
    }while ((!done_status) && (count > 0));
    
    ret_val = 1;
    if (done_status)
    {
        ret_val = 0;
    }
    else 
    {
        //platform_serial_flash_check_done_status: FAIL
    }
    
    return ret_val;
    
}

/******************************************************************************/
/** platform_serial_flash_process_cmd();
* Execute SPI Command
* @param cmd - SERIAL_FLASH_COMMAND
* @return qmspi_status - qmspi controller status
* @return command success (0) or error (1)
*******************************************************************************/
uint8_t platform_serial_flash_process_cmd (SERIAL_FLASH_COMMAND *cmd, uint32_t *qmspi_status)
{    
    uint8_t ret_val;

    //platform_serial_flash_process_cmd

    chipSelectSPI(SELECT); 
   
    if (api_qmspi_flash_cmd(cmd->write_len, cmd->write_buf, cmd->read_len, cmd->read_buf, NULL, spi_port_sel))
    {
        //platform_serial_flash_process_cmd: api_qmspi_flash_cmd FAIL
        *qmspi_status = 0;
        return 1;
    }
    
    ret_val = platform_serial_flash_check_done_status(qmspi_status);

    chipSelectSPI(DESELECT);  

    return ret_val;
}
//------------------------------------------------------------------------------

/******************************************************************************/
/** platform_serial_flash_read_dma();
* read the content from the SPI interface usign DMA interface. 
* Supported SPI Read commands are 
*       1 = 0x03 - Read data Single Output
*       2 = 0x0B - Fast Read Single Output
*       3 = 0x3B - Fast Read Dual Output
*       4 = 0x6B - Fast Read Quad Output
* @param spi_addr - SPI address to start reading from
* @param mem_addr - Memory address to copy data
* @param data_len - Number of bytes to read
* @param mode - Read mode
* @return command success (0) or error (1)
*******************************************************************************/
uint8_t platform_serial_flash_read_dma(uint32_t spi_addr, uint32_t mem_addr, uint32_t data_len, uint8_t mode, uint32_t *qmspi_status)
{
    
    uint32_t noofbytes;
    uint8_t ret_val, read_dma_sts;
    
    noofbytes = api_qmspi_flash_read24_dma(mode, spi_addr, data_len, mem_addr, spi_port_sel); 
    
    if (spi_port_sel == 0)
    {
        ret_val = api_dma_dev_xfr_cfg(DMA_CHANNEL_ID, QMSPI_RX_DMA_REQ_DIR0, mem_addr, noofbytes, DEV_ENABLE);
    }
    else
    {
        ret_val = api_dma_dev_xfr_cfg(DMA_CHANNEL_ID, QMSPI1_RX_DMA_REQ_DIR0, mem_addr, noofbytes, DEV_ENABLE);
    }   
    //ret_val = api_dma_dev_xfr_cfg(DMA_CHANNEL_ID, DMA_DEVICE_ID, mem_addr, noofbytes, DEV_ENABLE);
    if (!ret_val)
    {
        //platform_serial_flash_read_dma: api_dma_dev_xfr_cfg FAIL
        *qmspi_status = 0;
        read_dma_sts = 1;
    }
    else
    {
      chipSelectSPI(SELECT);  
      api_qmspi_start(0, spi_port_sel);
      read_dma_sts = platform_serial_flash_check_done_status(qmspi_status);
      chipSelectSPI(DESELECT);
    }
    return read_dma_sts;
}

/******************************************************************************/
/** platform_serial_flash_write();
* write data to SPI Flash
* @param spi_addr - SPI address to start writing from
* @param mem_addr - Memory address to copy data
* @param data_len - Number of bytes to write
* @return command success (0) or error (1)
*******************************************************************************/
uint8_t platform_serial_flash_write_page(uint32_t spi_addr,uint32_t mem_addr, uint32_t data_len, uint32_t *qmspi_status)
{    
    uint8_t ret_val;
   
    api_qmspi_flash_program_dma(0x02, spi_addr, data_len, spi_port_sel);
    
    if(0 == spi_port_sel)
    {
        ret_val = api_dma_dev_xfr_cfg(DMA_CHANNEL_ID, QMSPI_TX_DMA_REQ_DIR0, mem_addr, data_len, DEV_ENABLE);
    }
    else
    {
        ret_val = api_dma_dev_xfr_cfg(DMA_CHANNEL_ID, QMSPI1_TX_DMA_REQ_DIR0, mem_addr, data_len, DEV_ENABLE);
    }    
    //ret_val = api_dma_dev_xfr_cfg(DMA_CH07_ID, QMSPI_TX_DMA_REQ_DIR0, mem_addr, data_len, DEV_ENABLE);
    if (!ret_val)
    {
        *qmspi_status = 0;
        return 1;
    }
    
    chipSelectSPI(SELECT);
    api_qmspi_start(0, spi_port_sel);
       
    ret_val = platform_serial_flash_check_done_status(qmspi_status);

    chipSelectSPI(DESELECT);

    return ret_val;
}

