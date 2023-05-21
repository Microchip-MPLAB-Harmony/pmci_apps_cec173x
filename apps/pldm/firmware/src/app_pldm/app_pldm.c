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

#include "app_pldm.h"
#include "definitions.h"                // SYS function prototypes
#include "pldm/pldm.h"
#include "interrupt/interrupt_api.h"
#include "gpio/gpio_api.h"
#include "sb_gpio.h"
#include "rom_qmspi.h"
#include "platform_serial_flash.h"
#include "serial_flash_sst25pf.h"

uint32_t fw_staged_address;
uint8_t staged_spi_select;
uint32_t fw_active_address;
uint8_t active_spi_select;
uint16_t comp_id;

void pldm_get_firmware_param_resp_feilds(GET_FIRMWARE_PARAMETERS_RES_FIELDS *buf_ptr)
{
    staged_spi_select = SPI_SELECT_SHARED_COMP_0;
    active_spi_select = SPI_SELECT_INT_COMP_0;

    fw_staged_address = 0x480000 ; // shd spi
    fw_active_address = 0x20000 ;  // int spi 

    comp_id = 0x1121;
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
    buf_ptr->comp_parameter[0].comp_identifier = comp_id;
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
    if(component_id == comp_id)
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
    do
    {
        if(component_id == comp_id)
        {
            fw_staged_address += offset;
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
                timer_delay_ms(5);
            }

            if(sst25pf_program(fw_staged_address, (uint32_t)(buff_ptr), ONE_KB, &qmspi_sts))
            {
                ret_val = 1;
                break;
            }
        }
    } while (0);

    return ret_val;
}

uint8_t pldm_start_firmware_update(uint16_t component_id)
{
  return 0;
}


uint8_t pldm_start_firmware_apply()
{
  return 0;
}


uint8_t pldm_cancel_update(uint16_t component_id, uint8_t cancel_update_flag)
{
  return 0;
}


void  pldm_restore_configs(uint16_t component_id, uint8_t host_funct_reduced)
{

}


uint8_t pldm_reset_firmware_update_flags(void)
{
  return 0;
}


uint8_t pldm_activate_firmware(void)
{
  return 0;
}


void pldm_initiate_verify_req_tx_end()
{
}

void pldm_apply_complete_response()
{
}

uint8_t pldm_process_apply_complete(uint8_t apply_state)
{
  return 0;
}

void pldm_write_firmware_data_complete(uint16_t component_id)
{
}