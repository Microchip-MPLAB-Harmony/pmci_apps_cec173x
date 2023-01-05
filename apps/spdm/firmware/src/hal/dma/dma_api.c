/*****************************************************************************
* © 2022 Microchip Technology Inc. and its subsidiaries.
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
******************************************************************************

Version Control Information (Perforce)
******************************************************************************
$Revision: #1 $ 
$DateTime: 2022/11/28 03:10:21 $ 
$Author: i64652 $
Last Change:    None
******************************************************************************/
/** @file dma_api.c
* \brief DMA API Source file
* \author pramans
* 
* This file implements the DMA API functions  
******************************************************************************/

/** @defgroup DMA
 *  @{
 */

#include "common.h"
#include "dma_api.h"
#include "peripheral/dma/plib_dma.h"

/******************************************************************************/
/** get_device_id function.
 * This function retrieves the device id based on device name and instance
 * @param device_name - The device name - smb slave / smb master / spi flash
 * @return the device id
*******************************************************************************/
uint8_t get_device_id(const uint8_t device_name, const uint8_t device_instance)
{
	uint8_t device_id = 0;

	trace0(0, DMA, 0, "get_device_id: Enter ");

	switch (device_name)
	{
	case (uint8_t)SMBUS_SLAVE:
		device_id = (uint8_t)DEVICE_ID_SLAVE_1 + (uint8_t)(2U*device_instance);
		break;

	case (uint8_t)SMBUS_MASTER:
		device_id = (uint8_t)DEVICE_ID_MASTER_1 + (uint8_t)(2U*device_instance);
		break;

	case (uint8_t)SPI_FLASH:
		device_id = (uint8_t)DEVICE_ID_SPI_FLASH;
		break;

	default:
		trace0(0, DMA, 0, "get_device_id: invalid device name ");
		break;
	}

	return device_id;

}/* get_device_id */

/* end of dma_api.c */
/**   @}
 */

