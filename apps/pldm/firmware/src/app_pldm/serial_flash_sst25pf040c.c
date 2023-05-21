/*******************************************************************************
© 2017 Microchip Technology Inc. and its subsidiaries.  You may use this
software and any derivatives exclusively with Microchip products.

THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER EXPRESS,
IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES
OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE, OR
ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR
USE IN ANY APPLICATION.

IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER
RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF
THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT ALLOWED
BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS
SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY
TO MICROCHIP FOR THIS SOFTWARE.

MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
TERMS.

****************************************************************************/

#include <stdio.h>  
#include <stdlib.h>
#include <string.h>
#include "rom_qmspi.h"
#include "serial_flash_sst25pf.h"
#include "platform_serial_flash.h"
//------------------------------------------------------------------------------

extern uint8_t spi_port_sel;

/******************************************************************************/
/** sst25pf_read_status_register();
* Read Status Register (RDSR) - 0x05
* @return *status status register value
* @return command success (0) or error (1)
*******************************************************************************/
uint8_t sst25pf_read_status_register(uint8_t *status, uint32_t *qmspi_status)
{
    SERIAL_FLASH_COMMAND cmd;
    
    uint8_t cmd_buffer[4];
        
    memset(&cmd_buffer[0], 0, sizeof(cmd_buffer));
    
    cmd_buffer[0] = 0x05;
    
    cmd.write_buf = &cmd_buffer[0];
    cmd.write_len = 1;
    cmd.read_buf = status;
    cmd.read_len = 1;
    
    return platform_serial_flash_process_cmd(&cmd, qmspi_status); 
}

/******************************************************************************/
/** sst25pf_write_status_register();
* Write Status Register (RDSR) - 0x05
* @param value status register value
* @return command success (0) or error (1)
*******************************************************************************/
uint8_t sst25pf_write_status_register(uint8_t value, uint32_t *qmspi_status)
{
    SERIAL_FLASH_COMMAND cmd;
    
    uint8_t cmd_buffer[4]; 
       
    memset(&cmd_buffer[0], 0, sizeof(cmd_buffer));
    
    cmd_buffer[0] = 0x01;
    cmd_buffer[1] = value;
    
    cmd.write_buf = &cmd_buffer[0];
    cmd.write_len = 2;
    cmd.read_buf = NULL;
    cmd.read_len = 0;
    
    return platform_serial_flash_process_cmd(&cmd, qmspi_status);
   
}

/******************************************************************************/
/** sst25pf_memory_write_enable();
* Write Status Register (RDSR) - 0x05 with value 0 - to clear BPx bits
* @param None
* @return command success (0) or error (1)
*******************************************************************************/
uint8_t sst25pf_memory_write_enable(uint32_t *qmspi_status)
{
    SERIAL_FLASH_COMMAND cmd;
    uint8_t cmd_buffer[4];
    uint8_t value;    
    
    value = 0;
       
    memset(&cmd_buffer[0], 0, sizeof(cmd_buffer));
    
    cmd_buffer[0] = 0x01;
    cmd_buffer[1] = value;
    
    cmd.write_buf = &cmd_buffer[0];
    cmd.write_len = 2;
    cmd.read_buf = NULL;
    cmd.read_len = 0;
    
    return platform_serial_flash_process_cmd(&cmd, qmspi_status);
   
}

/******************************************************************************/
/** sst25pf_read_id();
* Read-ID 0xAB
* @return *jedec_id jedec read-id value
* @return command success (0) or error (1)
*******************************************************************************/
uint8_t sst25pf_read_id(uint8_t *read_id, uint32_t *qmspi_status)
{
    SERIAL_FLASH_COMMAND cmd;
    
    uint8_t cmd_buffer[4];
    
    //trace0(0, flash, 0, "sst25pf_read_id");
       
    memset(&cmd_buffer[0], 0, sizeof(cmd_buffer));
    
    cmd_buffer[0] = 0xAB;
    
    //Dummy address
    cmd_buffer[1] = 0x55;
    cmd_buffer[2] = 0xA5;
    cmd_buffer[3] = 0xAA;
    
    cmd.write_buf = &cmd_buffer[0];
    cmd.write_len = 4;
    cmd.read_buf = read_id;
    cmd.read_len = 1;
            
    return platform_serial_flash_process_cmd(&cmd, qmspi_status);
}

/******************************************************************************/
/** sst25pf_read_jedec_id();
* JEDEC Read-ID 0x9F
* @return *jedec_id jedec read-id value
* @return command success (0) or error (1)
*******************************************************************************/
uint8_t sst25pf_read_jedec_id(uint8_t *jedec_id, uint32_t *qmspi_status)
{
    SERIAL_FLASH_COMMAND cmd;
    
    uint8_t cmd_buffer[4];
    
    //trace0(0, flash, 0, "sst25pf_read_jedec_id");
       
    memset(&cmd_buffer[0], 0, sizeof(cmd_buffer));
    
    cmd_buffer[0] = 0x9F;
    
    cmd.write_buf = &cmd_buffer[0];
    cmd.write_len = 1;
    cmd.read_buf = (uint8_t *)jedec_id;
    cmd.read_len = 4;
            
    return platform_serial_flash_process_cmd(&cmd, qmspi_status);
}

/******************************************************************************/
/** sst25pf_write_enable();
* Write-Enable (WREN) - 0x06
* @param None
* @return command success (0) or error (1)
*******************************************************************************/
uint8_t sst25pf_write_enable(uint32_t *qmspi_status)
{
    SERIAL_FLASH_COMMAND cmd;
    
    uint8_t cmd_buffer[4];

    //trace0(0, flash, 0, "sst25pf_write_enable");
       
    memset(&cmd_buffer[0], 0, sizeof(cmd_buffer));
    
    cmd_buffer[0] = 0x06;
        
    cmd.write_buf = &cmd_buffer[0];
    cmd.write_len = 1;
    cmd.read_buf = NULL;
    cmd.read_len = 0;
    
    return platform_serial_flash_process_cmd(&cmd, qmspi_status);   
}

/******************************************************************************/
/** sst25pf_write_disable();
* Write-Disable (WRDI) - 0x04
* @param None
* @return command success (0) or error (1)
*******************************************************************************/
uint8_t sst25pf_write_disable(uint32_t *qmspi_status)
{
    SERIAL_FLASH_COMMAND cmd;
    
    uint8_t cmd_buffer[4];  

    //trace0(0, flash, 0, "sst25pf_write_disable");
       
    memset(&cmd_buffer[0], 0, sizeof(cmd_buffer));
    
    cmd_buffer[0] = 0x04;
        
    cmd.write_buf = &cmd_buffer[0];
    cmd.write_len = 1;
    cmd.read_buf = NULL;
    cmd.read_len = 0;
    
    return platform_serial_flash_process_cmd(&cmd, qmspi_status);   
}

/******************************************************************************/
/** sst25pf_chip_erase();
* Chip-Erase - 0x60 or 0xC7
* @param None
* @return command success (0) or error (1)
*******************************************************************************/
uint8_t sst25pf_chip_erase(uint32_t *qmspi_status)
{
    SERIAL_FLASH_COMMAND cmd;
    
    uint8_t cmd_buffer[4];

    //trace0(0, flash, 0, "sst25pf_chip_erase");
       
    memset(&cmd_buffer[0], 0, sizeof(cmd_buffer));
    
    cmd_buffer[0] = 0xC7;
        
    cmd.write_buf = &cmd_buffer[0];
    cmd.write_len = 1;
    cmd.read_buf = NULL;
    cmd.read_len = 0;
    
    return platform_serial_flash_process_cmd(&cmd, qmspi_status);   
}

/******************************************************************************/
/** sst25pf_sector_erase();
* Sector-Erase - 0x20 or 0xD7
* @param address sector address
* @return command success (0) or error (1)
*******************************************************************************/
uint8_t sst25pf_sector_erase(uint32_t address, uint32_t *qmspi_status)
{
    SERIAL_FLASH_COMMAND cmd;
    
    uint8_t cmd_buffer[4]; 

    //trace0(0, flash, 0, "sst25pf_sector_erase");
       
    memset(&cmd_buffer[0], 0, sizeof(cmd_buffer));
    
    //cmd_buffer[0] = 0xD7;
		cmd_buffer[0] = 0x20;
    cmd_buffer[1] = (address >> 16) & 0xFF;
    cmd_buffer[2] = (address >>  8) & 0xFF;
    cmd_buffer[3] = (address      ) & 0xFF;
        
    cmd.write_buf = &cmd_buffer[0];
    cmd.write_len = 4;
    cmd.read_buf = NULL;
    cmd.read_len = 0;
    
    return platform_serial_flash_process_cmd(&cmd, qmspi_status);   
}

/******************************************************************************/
/** sst25pf_block_erase();
* Block-Erase - 0xD8
* @param address block address
* @return command success (0) or error (1)
*******************************************************************************/
uint8_t sst25pf_block_erase(uint32_t address, uint32_t *qmspi_status)
{
    SERIAL_FLASH_COMMAND cmd;
    
    uint8_t cmd_buffer[4];   

    //trace0(0, flash, 0, "sst25pf_block_erase");
       
    memset(&cmd_buffer[0], 0, sizeof(cmd_buffer));
    
    cmd_buffer[0] = 0xD8;
    cmd_buffer[1] = (address >> 16) & 0xFF;
    cmd_buffer[2] = (address >>  8) & 0xFF;
    cmd_buffer[3] = (address      ) & 0xFF;
        
    cmd.write_buf = &cmd_buffer[0];
    cmd.write_len = 4;
    cmd.read_buf = NULL;
    cmd.read_len = 0;
    
    return platform_serial_flash_process_cmd(&cmd, qmspi_status);   
}

/******************************************************************************/
/** sst25pf_sector_erase_sequence();
* @param erase_address - Sector address
* @param qmspi_status - qmspi status
* @return command success (0) or error (1)
*******************************************************************************/
uint8_t sst25pf_sector_erase_sequence(uint32_t erase_address, uint32_t *qmspi_status)
{    
    uint8_t busy_status, ret_val=0;
    
    //trace0(0, flash, 0, "sst25pf_sector_erase_sequence");
    
    do
    {
        /*Global Block Protection Unlock*/
        if(sst25pf_global_protection_unlock_sequence(qmspi_status))
        {
            ret_val = 1;
            break;
        }
        
        /* Enable Write */
        if (sst25pf_write_enable(qmspi_status))
        {
            ret_val = 1;
            break;
        }        
  
        /* Sector Erase */
        if (sst25pf_sector_erase(erase_address, qmspi_status))
        {
            ret_val = 1;
            break;
        }        
                    
        /* Wait for Erase to complete */
        timer_delay_ms(SECTOR_ERASE_TYP_TIME_MS);
        
        /* Check busy bit by polling */
        ret_val = sst25pf_check_busy(&busy_status, 
                                    SECTOR_ERASE_CHECK_BUSY_TIME_MS, 
                                    SECTOR_ERASE_CHECK_BUSY_POLL_MS, 
                                    qmspi_status);
        if (ret_val || busy_status)
        {
            ret_val = 1;
            break;
        } 
        
    } while (0);
    
    return ret_val;    
}

/******************************************************************************/
/** sst25pf_block_erase_sequence();
* @param erase_address - block address
* @param qmspi_status - qmspi status
* @return command success (0) or error (1)
*******************************************************************************/
uint8_t sst25pf_block_erase_sequence(uint32_t erase_address, uint32_t *qmspi_status)
{    
    uint8_t busy_status, ret_val=0;
    
    //trace0(0, flash, 0, "sst25pf_block_erase_sequence");
    
    do
    {
        /*Global Block Protection Unlock*/
        if(sst25pf_global_protection_unlock_sequence(qmspi_status))
        {
            ret_val = 1;
            break;
        }
        
        /* Enable Write */
        if (sst25pf_write_enable(qmspi_status))
        {
            ret_val = 1;
            break;
        }        
  
        /* Block Erase */
        if (sst25pf_block_erase(erase_address, qmspi_status))
        {
            ret_val = 1;
            break;
        }        
                    
        /* Wait for Erase to complete */
        timer_delay_ms(BLOCK_ERASE_TYP_TIME_MS);
        
        /* Check busy bit by polling */
        ret_val = sst25pf_check_busy(&busy_status, 
                                    BLOCK_ERASE_CHECK_BUSY_TIME_MS, 
                                    BLOCK_ERASE_CHECK_BUSY_POLL_MS, 
                                    qmspi_status);
        if (ret_val || busy_status)
        {
            ret_val = 1;
            break;
        } 
        
    } while (0);
    
    return ret_val;    
}

/******************************************************************************/
/** sst25pf_chip_erase_sequence();
* @param qmspi_status - qmspi status
* @return command success (0) or error (1)
*******************************************************************************/
uint8_t sst25pf_chip_erase_sequence(uint32_t *qmspi_status)
{    
    uint8_t busy_status, ret_val=0, bufftx[2], buffrx[2];
		uint32_t qstatus, max_erase_time = MAX_ERASE_TIME;
    
    //trace0(0, flash, 0, "sst25pf_chip_erase_sequence");
    
    do
    {
        /*Global Block Protection Unlock*/
        if(sst25pf_global_protection_unlock_sequence(qmspi_status))
        {
            ret_val = 1;
            break;
        }
        
        /* Enable Write */
        if (sst25pf_write_enable(qmspi_status))
        {
            ret_val = 1;
            break;
        }        
  
        /* Chip Erase */
        if (sst25pf_chip_erase(qmspi_status))
        {
            ret_val = 1;
            break;
        }        
                    
        /* Wait for Erase to complete */
				/*polling for BUSY status bit */
				bufftx[0]=0x5;
				buffrx[0]=0x0;
                chipSelectSPI(SELECT);
				api_qmspi_flash_cmd(1,&bufftx[0], 1, &buffrx[0], NULL, spi_port_sel);
				while (!api_qmspi_is_done_status(&qstatus, spi_port_sel));
                chipSelectSPI(DESELECT);
				
				/* Check busy status bit of status register and wait for the erase 
         * operation to be completed.
				 * Wait for 20 sec max */
				
				while(max_erase_time && (buffrx[0] & STATUS_BUSY_BIT_MASK))
				{
					timer_delay_ms(CHIP_ERASE_TYP_TIME_MS);
                    chipSelectSPI(SELECT);
					api_qmspi_flash_cmd(1,&bufftx[0], 1, &buffrx[0], NULL, spi_port_sel);
					while (!api_qmspi_is_done_status(&qstatus, spi_port_sel));
                    chipSelectSPI(DESELECT);
					max_erase_time = max_erase_time - CHIP_ERASE_TYP_TIME_MS;	
				}

        /* Check busy bit by polling */
        ret_val = sst25pf_check_busy(&busy_status, 
                                    CHIP_ERASE_CHECK_BUSY_TIME_MS, 
                                    CHIP_ERASE_CHECK_BUSY_POLL_MS, 
                                    qmspi_status);
        if (ret_val || busy_status)
        {
            ret_val = 1;
            break;
        } 
        
    } while (0);
    
    return ret_val;    
}

/******************************************************************************/
/** sst25pf_program();
* write data to SPI Flash
* @param spi_addr - SPI address to start writing from
* @param mem_addr - Memory address to copy data
* @param data_len - Number of bytes to write
* @return command success (0) or error (1)
*******************************************************************************/
uint8_t sst25pf_program(uint32_t spi_addr, uint32_t mem_addr, uint32_t data_len, uint32_t *qmspi_status)
{    
    uint32_t next_aligned_page_addr, remaining_size,*q_status;
    uint16_t page_program_len;    
    uint8_t busy_status, ret_val=0;
    
    if (data_len < PAGE_SIZE)
    {
        page_program_len = data_len;
        remaining_size = data_len;
    }
    else
    {
    
    next_aligned_page_addr = (spi_addr & PAGE_ADDRESS_ALIGN_MASK) + PAGE_SIZE;
    page_program_len = next_aligned_page_addr - spi_addr;   
    remaining_size = data_len;
    }
    
    ////trace0(467, flash, 0, "sst25pf_program: spi_addr");
    //trace14(0, platform, 0, "sst25pf_program: spi_addr: 0x%08X data_len: 0x%08X nxtAlignedPAddr: 0x%08X curPgPrgLen: 0x%08X", spi_addr, data_len, next_aligned_page_addr, page_program_len);
    
    do
    {    
        if (sst25pf_memory_write_enable(qmspi_status))
        {            
            ret_val = 1;
            break;                
        }
        
        do 
        {
					
					
            /* Enable Write */
            if (sst25pf_write_enable(qmspi_status))
            {
                ret_val = 1;
                break;
            }            
            /* Write Page */

            if (platform_serial_flash_write_page(spi_addr, mem_addr, page_program_len, qmspi_status))
            {
                    ret_val = 1;
                    break;
            }                
        						
            remaining_size = remaining_size - page_program_len;
            
            mem_addr += page_program_len;            
            spi_addr = next_aligned_page_addr;
            
            page_program_len = PAGE_SIZE;        
            if (remaining_size < PAGE_SIZE)
            {
                page_program_len = remaining_size;
            }
            
            next_aligned_page_addr += PAGE_SIZE;
            
            
            /* Wait for Page Program to complete */
            timer_delay_ms(PAGE_PROGRAM_TYP_TIME_MS);
            
            /* Check busy bit by polling */
            ret_val = sst25pf_check_busy(&busy_status, 
                                        PAGE_PROGRAM_CHECK_BUSY_TIME_MS, 
                                        PAGE_PROGRAM_CHECK_BUSY_POLL_MS, 
                                        qmspi_status);
            if (ret_val || busy_status)
            {
                ret_val = 1;
                break;
            }
            
        } while (remaining_size > 0);
        
    } while (0);
    
    
    /* Flash write disabled*/
    if(sst25pf_write_disable(q_status))
    {
        //trace0(0, platform, 0, "write disable fail");
    }
    else 
    {
        //trace0(0, platform, 0, "write disable success");
    }
    
    
    return ret_val;
    
}

/******************************************************************************/
/** sst25pf_check_busy();
* @param qmspi_status   qmspi operation status
* @param timeout_ms  time to poll for checking busy to clear
* @param read_interval_ms  interval between status read during polling
* @return busy_status 1 if sst25pf busy, else 0
* @return command success (0) or error (1)
*******************************************************************************/
uint8_t sst25pf_check_busy(uint8_t *busy_status, uint16_t timeout_ms, uint16_t read_interval_ms, uint32_t *qmspi_status)
{
    uint8_t read_status, ret_val=0;
    uint16_t delay_ms;
    
    *busy_status = 1;
    
    //trace0(0, flash, 0, "sst25pf_check_busy");
    
    do
    {        
        /* Read Status Register */
        if (sst25pf_read_status_register(&read_status, qmspi_status))
        {
            ret_val = 1;
            break;
        }    
            
        /* Check busy bit */
        if (!(read_status & STATUS_BUSY_BIT_MASK))
        {
            *busy_status = 0;
            break;
        }
        
        delay_ms = read_interval_ms;
        if (timeout_ms < read_interval_ms)
        {
            delay_ms = timeout_ms;
        }
        
        timer_delay_ms(delay_ms);
        
        timeout_ms = timeout_ms - delay_ms;
        
    } while (timeout_ms);
    
    return ret_val;
  
}

/******************************************************************************/
/** sst25pf_global_protection_unlock();
* Global Block Protection Unlock(ULBPR) - 0x98
* @param qmspi_status   qmspi operation status
* @return command success (0) or error (1)
*******************************************************************************/
uint8_t sst25pf_global_protection_unlock(uint32_t *qmspi_status)
{   
    SERIAL_FLASH_COMMAND cmd;
    uint8_t cmd_buffer[4];
    uint8_t value;    

    //trace0(0, flash, 0, "Global Protection Unlock");
    
    value = 0;
       
    memset(&cmd_buffer[0], 0, sizeof(cmd_buffer));
    
    cmd_buffer[0] = 0x98;
    cmd.write_buf = &cmd_buffer[0];
    cmd.write_len = 1;
    cmd.read_buf = NULL;
    cmd.read_len = 0;
    
    return platform_serial_flash_process_cmd(&cmd, qmspi_status);     
}

/******************************************************************************/
/** sst25pf_global_protection_unlock_sequence();
* @param qmspi_status   qmspi operation status
* @return command success (0) or error (1)
*******************************************************************************/
uint8_t sst25pf_global_protection_unlock_sequence(uint32_t *qmspi_status)
{
    uint8_t ret_val = 0;
    uint8_t retry_count = 0;
    uint8_t max_retry_count = 5;
    uint8_t wel_status = 0, busy = 0;
    do 
    {
        /* Enable Write */
        if (sst25pf_write_enable(qmspi_status))
        {
            ret_val = 1;
            break;
        }
        
        /*Send Unlock Global Block Protection Command */
        if (sst25pf_global_protection_unlock(qmspi_status))
        {
            ret_val =1;
            break;
        }

        /* Check busy bit by polling */
        ret_val = sst25pf_check_busy(&busy, ULBPR_TIMEOUT_MS, ULBPR_POLL_TIME_MS, qmspi_status);
        if(busy || ret_val)
        {
            ret_val =1;
        }
        
    }while(0);
    
    return ret_val;
}


//------------------------------------------------------------------------------

