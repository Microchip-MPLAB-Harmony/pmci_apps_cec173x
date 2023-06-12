/*******************************************************************************
  Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app_pldm.c

  Summary:
    This file contains the "main" function for a project.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It
    implements the logic of the application's state machine and it may call
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

#include "pldm_app.h"
#include "definitions.h"                // SYS function prototypes
#include "pldm/pldm.h"
#include "interrupt/interrupt_api.h"
#include "gpio/gpio_api.h"
#include "sb_gpio.h"
#include "rom_qmspi.h"
#include "platform_serial_flash.h"
#include "serial_flash_sst25pf.h"

#define HEADER_SIZE 896u
#define COMPONENT_IDENTIFIER 0x1020
#define STAGED_ADDR        0x480000
#define ACTIVE_ADDR        0x20000


uint32_t fw_staged_address;
uint8_t staged_spi_select;
uint32_t fw_active_address;
uint8_t active_spi_select;

// off set of image in the firmware data
uint32_t offset_tagx;
uint32_t staged_image_size;
uint32_t active_image_size;
uint8_t cancel_state;
uint8_t verify_state;
uint8_t apply_state;

// Hash of image in header 
uint8_t header_hash_buffer[48];
// Calculated hash of image 
uint8_t image_hash_buffer[48];

// Spi read for intermidiate hashing 
uint8_t spi_data[1024]__attribute__((aligned(8)));

uint8_t statebuf[256];

enum PLDM_APP_STATES
{
  PLDM_APP_STATE_INIT,
  PLDM_APP_STATE_STAGED_AUTH,
  PLDM_APP_STATE_COPY_STAGED_TO_ACTIVE,
  PLDM_APP_STATE_ACTIVE_AUTH,
  PLDM_APP_STATE_SUCCESS,
  PLDM_APP_STATE_FAIL
};

uint8_t pldm_app_state = PLDM_APP_STATE_INIT;

#define TEST_PLDM_APP 1

/******************************************************************************/
/** sb_validate_hash();
* Compare Input buffer with compare buffer of provided length
* @param input_buf - pointer to input buffer
* @param input_len - length of input buffer
* @param compare_buf - pointer to hash buffer to compared with
* @return 0 on success; 1 on failure
*******************************************************************************/
uint8_t sb_validate_hash(  uint8_t *input_buf, uint32_t length, uint8_t *compare_buf )
{
    uint8_t return_val = 0u;
    do
    {
        if((NULL == input_buf) || (NULL == compare_buf))
        {
            return_val  = 1;
            break;
        }
        if((memcmp(&input_buf[0], &compare_buf[0], (size_t) length)==0))
        {
            //Validate hash:Dgst succ

        }
        else
        {
            //Validate hash:Dgst fail
            return_val = 1;
        }
    }
    while(0);

    return return_val;
}

bool staged_image_auth()
{
      uint32_t q_st;
      // read header to get image size and hash from header 
      
      spi_port_sel_set(SHD_SPI);
      component_set(0);
      spi_init_shd_spi();
      platform_serial_flash_read_dma(STAGED_ADDR, (uint32_t)(&spi_data[0]), 
                                HEADER_SIZE, QMSPI_SPI_SINGLE, &q_st);
      memcpy(&(offset_tagx), &(spi_data[0x14]), 4);
      fw_staged_address = STAGED_ADDR + offset_tagx;
      memcpy(&staged_image_size, &(spi_data[0x10]), 2);
      staged_image_size = (staged_image_size * 128) + 128;

      memcpy(&header_hash_buffer[0], &(spi_data[0x200]), 48);
      
      // authenticate image
      struct mchphash c;
      struct mchphashstate hstate;
      uint32_t i;
      int r;
      memset(statebuf, 0, sizeof(statebuf));
      uint32_t remaining_byte =0, length = 0;
      r = mchp_hash_create_sha384(&c);
      mchp_hash_init_state(&c, &hstate, (char *)statebuf);
      remaining_byte = staged_image_size;
       
      uint32_t ec_fw_size_in_1k = staged_image_size / ONE_KB;
      if (staged_image_size % ONE_KB)
      {
        ec_fw_size_in_1k = ec_fw_size_in_1k + 1;
      }

      for (i = 0; i < ec_fw_size_in_1k; i++)
      {
        length = ONE_KB;

        if (remaining_byte < ONE_KB)
        {
            length = remaining_byte;
        }

        spi_port_sel_set(SHD_SPI);
        component_set(0);
        if (platform_serial_flash_read_dma(fw_staged_address, (uint32_t)(&spi_data[0]),
                                        length, QMSPI_SPI_SINGLE, &q_st))
        {
            return true;
        }
        if (i != 0)
        {
            r = mchp_hash_create_sha384(&c);
        }

        r = mchp_hash_resume_state(&c, & hstate);
        if (r != 0)
        {
            return r;
        }
        //Assign data to be hashed.
        r = mchp_hash_feed(&c,(const char*)&spi_data[0], length);

        if(r == -69)
        {
            return true;
        }

        // Exports the state of partial hashing.
        r = mchp_hash_save_state(&c);

        if (r == -70)
        {
            return true;
        }

        //Wait until the hashing finishes.
        mchp_hash_wait(&c);
        fw_staged_address = fw_staged_address + length;
        remaining_byte = remaining_byte - length;
     }
     sb_spi_tristate();

    if (i != 0)
    {
        mchp_hash_create_sha384(&c);
    }

    mchp_hash_resume_state(&c, &hstate);
    mchp_hash_digest(&c, (char *)&image_hash_buffer[0]);
    r = mchp_hash_wait(&c);
    if (r != 0)
    {
        return true;
    }
    if(sb_validate_hash(image_hash_buffer, 48, header_hash_buffer))
    {
      return true;
    }
}

bool active_image_auth()
{
      uint32_t q_st;
      // read header to get image size and hash from header
      //sb_spi_tristate(); 
      spi_port_sel_set(INT_SPI);
      component_set(0);
      spi_init_int_spi();
      platform_serial_flash_read_dma(ACTIVE_ADDR, (uint32_t)(&spi_data[0]), 
                                HEADER_SIZE, QMSPI_SPI_SINGLE, &q_st);
      memcpy(&(offset_tagx), &(spi_data[0x14]), 4);
      fw_active_address = ACTIVE_ADDR + offset_tagx;
      memcpy(&active_image_size, &(spi_data[0x10]), 2);
      active_image_size = (active_image_size * 128) + 128;
      active_image_size = active_image_size;

      memcpy(&header_hash_buffer[0], &(spi_data[0x200]), 48);
      
      // authenticate image
      struct mchphash c;
      struct mchphashstate hstate;
      uint32_t i;
      int r;
      memset(statebuf, 0, sizeof(statebuf));
      uint32_t remaining_byte =0, length = 0;
      r = mchp_hash_create_sha384(&c);
      mchp_hash_init_state(&c, &hstate, (char *)statebuf);
      remaining_byte = active_image_size;
       
      uint32_t ec_fw_size_in_1k = active_image_size / ONE_KB;
      if (active_image_size % ONE_KB)
      {
        ec_fw_size_in_1k = ec_fw_size_in_1k + 1;
      }

      for (i = 0; i < ec_fw_size_in_1k; i++)
      {
        length = ONE_KB;

        if (remaining_byte < ONE_KB)
        {
            length = remaining_byte;
        }

        spi_port_sel_set(INT_SPI);
        component_set(0);
        if (platform_serial_flash_read_dma(fw_active_address, (uint32_t)(&spi_data[0]),
                                        length, QMSPI_SPI_SINGLE, &q_st))
        {
            return true;
        }
        if (i != 0)
        {
            r = mchp_hash_create_sha384(&c);
        }

        r = mchp_hash_resume_state(&c, & hstate);
        if (r != 0)
        {
            return r;
        }
        //Assign data to be hashed.
        r = mchp_hash_feed(&c,(const char*)&spi_data[0], length);

        if(r == -69)
        {
            return true;
        }

        // Exports the state of partial hashing.
        r = mchp_hash_save_state(&c);

        if (r == -70)
        {
            return true;
        }

        //Wait until the hashing finishes.
        mchp_hash_wait(&c);
        fw_active_address = fw_active_address + length;
        remaining_byte = remaining_byte - length;
     }
     sb_spi_tristate();

    if (i != 0)
    {
        mchp_hash_create_sha384(&c);
    }

    mchp_hash_resume_state(&c, &hstate);
    mchp_hash_digest(&c, (char *)&image_hash_buffer[0]);
    r = mchp_hash_wait(&c);
    if (r != 0)
    {
        return true;
    }
    if(sb_validate_hash(image_hash_buffer, 48, header_hash_buffer))
    {
//      printf("\n\r vld fl");
      return true;
    }
}

void pldm_get_firmware_param_resp_feilds(GET_FIRMWARE_PARAMETERS_RES_FIELDS *buf_ptr)
{
    staged_spi_select = SPI_SELECT_SHARED_COMP_0;
    active_spi_select = SPI_SELECT_INT_COMP_0;

    fw_staged_address = STAGED_ADDR ; // shd spi
    fw_active_address = ACTIVE_ADDR ;  // int spi 

    uint8_t ver[ASCII_SIZE] = "IMAGE0";
    uint8_t ver_pend[ASCII_SIZE] = "IMAGE1";
    buf_ptr->component_count = 1;
    buf_ptr->active_comp_image_set_version_string_type = ASCII;
    buf_ptr->active_comp_image_set_version_string_length = ASCII_SIZE;
    buf_ptr->pending_comp_image_set_version_string_type = ASCII;
    buf_ptr->pending_comp_image_set_version_string_length = ASCII_SIZE;  
    memcpy(buf_ptr->active_comp_image_set_version_string, ver, ASCII_SIZE);
    memcpy(buf_ptr->pending_comp_image_set_version_string, ver_pend, ASCII_SIZE);
    buf_ptr->comp_parameter[0].comp_classification = PLDM_COMP_CLASSIFICATION;
    buf_ptr->comp_parameter[0].comp_identifier = COMPONENT_IDENTIFIER;
    buf_ptr->comp_parameter[0].comp_classification_index = 0x00;
    buf_ptr->comp_parameter[0].active_comp_comparison_stamp = 0x00000000;
    buf_ptr->comp_parameter[0].active_comp_version_string_type = UTF16;
    buf_ptr->comp_parameter[0].active_comp_version_string_length = COMP_STRING_TYPE_SIZE;
    memset(buf_ptr->comp_parameter[0].active_comp_release_date, 0, 8);
    buf_ptr->comp_parameter[0].pending_comp_comparison_stamp = 0x00000000;
    buf_ptr->comp_parameter[0].pending_comp_version_string_type = UTF16;
    buf_ptr->comp_parameter[0].pending_comp_version_string_length = COMP_STRING_TYPE_SIZE;
    memset(buf_ptr->comp_parameter[0].pending_comp_release_date, 0, 8);

    buf_ptr->comp_parameter[0].comp_activation_methods = PLDM_COMP_ACTIVATION_SUPPORTED;
    buf_ptr->comp_parameter[0].cap_during_update = 0x00000000;
    uint16_t FW_build_actv = 0x0100;
    uint16_t FW_build_pend = 0x0101;
    memcpy(buf_ptr->comp_parameter[0].active_comp_version_string, &FW_build_actv, COMP_STRING_TYPE_SIZE);
    memcpy(buf_ptr->comp_parameter[0].pending_comp_version_string, &FW_build_pend, COMP_STRING_TYPE_SIZE);
}

void pldm_init_peripheral_for_update(uint16_t component_id)
{
    sb_spi_tristate();  
    if(component_id == COMPONENT_IDENTIFIER)
    {
        if(SPI_SELECT_SHARED_COMP_0 == staged_spi_select)
        {
            spi_port_sel_set(SHD_SPI);
            spi_init_shd_spi();
        }
        else if(SPI_SELECT_INT_COMP_0 == staged_spi_select)
        {
            spi_port_sel_set(INT_SPI);
            spi_init_int_spi();
        }
    }
}

uint8_t pldm_write_firmware_data(uint16_t component_id, uint8_t *buff_ptr, uint32_t offset)
{
    uint8_t ret_val = 0;
#if TEST_PLDM_APP    
    do
    {
        if(component_id == COMPONENT_IDENTIFIER)
        {
            fw_staged_address = STAGED_ADDR + offset ;
            uint32_t qmspi_sts;
            spi_port_sel_set(SHD_SPI);
            component_set(0);
            if(!(offset % SERIAL_FLASH_SECTOR_SIZE))
            {
                if(sst25pf_sector_erase_sequence(fw_staged_address, &qmspi_sts))
                {
                    ret_val = 1;
                    break;
                }
            }

            if(sst25pf_program(fw_staged_address, (uint32_t)(buff_ptr), ONE_KB, &qmspi_sts))
            {
                ret_val = 1;
                break;
            }
        }
    } while (0);
#endif
    return ret_val;
}

void pldm_start_firmware_update(uint16_t component_id)
{
    uint32_t q_st = 0;
    cancel_state = false;

    
    verify_state = 1;
#if TEST_PLDM_APP
    if(COMPONENT_IDENTIFIER == component_id)
    {
        if(PLDM_APP_STATE_STAGED_AUTH == pldm_app_state)
        {
         // printf("\n\r Staged Auth");
            if(staged_image_auth())
            {
                // PLDM_VERIFY_FAILURE
                verify_state = 1;
        
            }
            else
            {
                // Pldm Staged authentication success
                verify_state = 0;
              //  printf("\n\r Vrfy sucs");
                pldm_app_state = PLDM_APP_STATE_COPY_STAGED_TO_ACTIVE;
            }
        }
    
    }
#else
    verify_state = 0;
#endif
    pldm_initiate_verify_req_to_update_agent(verify_state);
}


void pldm_start_firmware_apply()
{
    // copy staged to active 
    // Auth Active
    // apply success 
    apply_state = 1;
#if TEST_PLDM_APP
    if(!verify_state)
    {
        uint8_t ret_val = 0;
        uint32_t remaining_bytes = HEADER_SIZE + staged_image_size;
        uint32_t offset = 0;
        if(PLDM_APP_STATE_COPY_STAGED_TO_ACTIVE ==
          pldm_app_state)
        {
          uint32_t qmspi_sts;
          uint32_t length = ONE_KB;

          do
          {
              if(remaining_bytes < length)
              {
                length = remaining_bytes;
              }

              //sb_spi_tristate();
              spi_port_sel_set(SHD_SPI);
              component_set(0);
              spi_init_shd_spi();
              fw_staged_address = STAGED_ADDR + offset;
              if (platform_serial_flash_read_dma(fw_staged_address, (uint32_t)(&spi_data[0]),
                                      length, QMSPI_SPI_SINGLE, &qmspi_sts))
              {
                  //printf("\n\r ER1");
                  ret_val = 1;
                  break;
              }
              
              //sb_spi_tristate();
              spi_port_sel_set(INT_SPI);
              component_set(0);
              spi_init_int_spi();
              fw_active_address = ACTIVE_ADDR + offset ;
              if(!(offset % SERIAL_FLASH_SECTOR_SIZE))
              {
                  if(sst25pf_sector_erase_sequence(fw_active_address, &qmspi_sts))
                  {
                     // printf("\n\r ER2");
                      ret_val = 1;
                      break;
                  }
              }

              if(sst25pf_program(fw_active_address, (uint32_t)(&spi_data[0]), length, &qmspi_sts))
              {
                 // printf("\n\r ER3");
                  ret_val = 1;
                  break;
              }

              offset += length;
              remaining_bytes = remaining_bytes - length;
          }while(remaining_bytes);

          if(!ret_val)
          {
              // Active copied
            //  printf("\n\r Act FxS");
              pldm_app_state = PLDM_APP_STATE_ACTIVE_AUTH;
              
              if(active_image_auth())
              {
                  // Active Auth fail
                  pldm_app_state = PLDM_APP_STATE_FAIL;
              }
              else
              {
                  apply_state = 0;
                 // printf("\n\r Actv vrf Scs");
                  pldm_app_state = PLDM_APP_STATE_SUCCESS;
              }
          }
        }
    }
#else 
    apply_state = 0;
#endif
    pldm_initiate_apply_req_to_update_agent(apply_state);
}


uint8_t pldm_cancel_update(uint16_t component_id, uint8_t cancel_update_flag)
{
    cancel_state = true;
    return 0;
}


void  pldm_restore_configs(uint16_t component_id, uint8_t host_funct_reduced)
{
    // spi tristate
}


void pldm_reset_firmware_update_flags(void)
{
    //NOP
}


void pldm_activate_firmware(void)
{
    // sys reset
    //printf("\n\r sysreset");
     PCR_REGS->PCR_SYS_RST |= PCR_SYS_RST_SOFT_SYS_RST(TRUE);
}


void pldm_initiate_verify_req_tx_end()
{
    // NOP
}

void pldm_write_firmware_data_complete(uint16_t component_id)
{
    //tristate spi
    sb_spi_tristate();
    pldm_app_state = PLDM_APP_STATE_STAGED_AUTH;
}