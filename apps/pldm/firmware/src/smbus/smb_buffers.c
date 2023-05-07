/*****************************************************************************
* © 2013 Microchip Technology Inc. and its subsidiaries.
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

/** @file smb_buffers.c
 *MEC1322 Boot ROM SMBus buffers
 */
/** @defgroup bootrom smbus
 *  @{
 */


/******************************************************************************
 *  MCHP Version Control Information (Perforce):
 *
 *  FILE:     $File: //depot_pcs/FWEng/solutions/Glacier_GEN3/maincodeline/Harmony/pmci_stack/pmci_apps_cec173x/apps/spdm/firmware/src/smbus/smb_buffers.c $
 *  REVISION: $Revision: #2 $
 *  DATETIME: $DateTime: 2023/02/24 06:07:07 $
 *  AUTHOR:   $Author: i64652 $
 *
 *  Revision history (latest first):
 *  #1  2013/09/10  scottw  initial version 
 ******************************************************************************
*/



#include "common.h"
#include "smb_config.h"


/* Moved from smb_core.cpp so we can locate just the buffers with the linker */
UINT8  __attribute__((section("smbBufBlock1"), aligned(SMB_BB1_ALIGN)))
bufferPool1[SMB_MAX_NUM_SLAVE_BUFFER * SLAVE1_BUFFER_SIZE];
#if MAX_SMB > 1
UINT8 __attribute__((section("smbBufBlock2"), aligned(SMB_BB2_ALIGN)))
bufferPool2[SMB_MAX_NUM_SLAVE_BUFFER * SLAVE2_BUFFER_SIZE];
#endif
#if MAX_SMB > 2
UINT8 __attribute__((section("smbBufBlock3"), aligned(SMB_BB3_ALIGN)))
bufferPool3[SMB_MAX_NUM_SLAVE_BUFFER * SLAVE3_BUFFER_SIZE];
#endif
#if MAX_SMB > 3
UINT8 __attribute__((section("smbBufBlock4"), aligned(SMB_BB4_ALIGN)))
bufferPool4[SMB_MAX_NUM_SLAVE_BUFFER * SLAVE4_BUFFER_SIZE];
#endif
#if MAX_SMB > 4
UINT8 __attribute__((section("smbBufBlock5"), aligned(SMB_BB5_ALIGN)))
bufferPool5[SMB_MAX_NUM_SLAVE_BUFFER * SLAVE5_BUFFER_SIZE];
#endif

/* end smb_buffers.c */
/**   @}
 */
