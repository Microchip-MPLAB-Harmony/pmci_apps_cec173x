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

#ifndef SPDM_TASK_H
#define SPDM_TASK_H

#include <stddef.h>
#include <stdint.h>
#include "spdm_common.h"
#include "mctp_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define configSPDM_PRIORITY (MCTP_TASK_PRIORITY - 1)
#define SPDM_PRIORITY ((tskIDLE_PRIORITY + configSPDM_PRIORITY) % configMAX_PRIORITIES)

/* Stack size must be a power of 2 if the task is restricted */
#define SPDM_STACK_SIZE 2048       // 2 * configMINIMAL_STACK_SIZE (120)
#define SPDM_STACK_WORD_SIZE ((SPDM_STACK_SIZE) / 2U)

#define SPDM_STACK_ALIGN __attribute__ ((aligned(SPDM_STACK_SIZE)))

#define SPDM_TASK_BUF_SIZE 512U
#define SPDM_TASK_BUF_MPU_ATTR 0U

#define SPDM_EVENT_BIT                  (1 << 0u)
#define SPDM_GET_FROM_APCFG_EVENT_BIT   (1 << 1u)
#define SPDM_POST_AUTH_DONE_BIT         (1 << 2u)
#define PLDM_EVENT_BIT                  (1 << 3u)
#define SPDM_I2C_EVENT_BIT              (1 << 4u)
#define PLDM_RESP_EVENT_BIT             (1 << 5u)

int spdm_task_create(void *pvParams);

#define SPDM_TASK_LOG_MBOX_SIZE                    0x2000u
#define SPDM_TASK_LOG_MBOX_START_ADDR              0x126000u
#define SPDM_TASK_LOG_MBOX_MPU_ATTRIB              (portMPU_REGION_READ_ONLY | portMPU_REGION_EXECUTE_NEVER)

#define SPDM_TASK_BUF_ALIGN __attribute__((aligned(SPDM_TASK_BUF_SIZE)))

/** SPDM Task Modes */
enum SPDM_TASK_MODES
{
    SPDM_IDLE,
    SPDM_INIT_CERT_PARAMS,   // Initialization
    SPDM_GET_CERT_FROM_APCFG,
    SPDM_COPY_CERT_DATA_TO_BUF,
    SPDM_CALC_HASH_CHAIN,
    SPDM_CMD_PROCESS_MODE,
    PLDM_IDLE,
    PLDM_CMD_PROCESS_MODE,
    PLDM_CMD_GET_AP_CFG
};


/******************************************************************************/
/**  SPDM Context Information
*******************************************************************************/
typedef struct SPDM_CONTEXT
{
    uint8_t spdm_state_info;

    uint8_t current_resp_cmd;

    uint8_t challenge_success_flag;

    uint8_t host_eid;

    uint8_t ec_eid;

    uint8_t ec_slv_addr;

    uint8_t host_slv_addr;

    uint8_t message_tag;

    uint8_t spdm_cmd_code;

    /* Event group handle */
    EventGroupHandle_t xSPDMEventGroupHandle;

    /* Event group buffer*/
    StaticEventGroup_t xSPDMCreatedEventGroup;

    uint8_t no_of_chains_avail;

    uint8_t previous_state;

    uint8_t get_requests_state;

    uint8_t requested_slot[8];

    uint8_t sha_digest[48];

    uint8_t request_or_response;

    uint16_t current_request_length;

    uint16_t previous_request_length;

    uint16_t total_request_bytes;

    uint16_t cert_offset_to_read[8];

    uint16_t cert_bytes_to_sent_curr[8];

    uint16_t cert_bytes_pending_to_sent[8];

    uint16_t cert_bytes_requested[8]  __attribute__((aligned(8)));
  
} __attribute__((packed)) SPDM_CONTEXT;

void spdm_init_task(SPDM_CONTEXT *spdmContext);
void spdm_pkt_initialize_cert_params_to_default(SPDM_CONTEXT *spdmContext);
void spdm_pkt_get_cert_from_apcfg(SPDM_CONTEXT *spdmContext);
void spdm_pkt_copy_cert_data_to_buf(SPDM_CONTEXT *spdmContext);
void spdm_pkt_store_hash_of_chain(SPDM_CONTEXT *spdmContext);
SPDM_CONTEXT* spdm_ctxt_get(void);
/******************************************************************************/
/** This function will periodically request for the status of post authentication
* completion from sb_core task (trigger period = 500ms)
* @param none
* @return void
*******************************************************************************/
void spdm_wait_post_auth_completion(SPDM_CONTEXT *spdmContext);


#ifdef __cplusplus
}
#endif
#endif