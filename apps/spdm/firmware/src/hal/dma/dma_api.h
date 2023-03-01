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
$Revision: #2 $ 
$DateTime: 2023/02/24 06:07:07 $ 
$Author: i64652 $
Last Change:    None
******************************************************************************/
/** @file dma_api.h
* \brief DMA Peripheral Header File
* \author pramans
* 
* This file is the header file for DMA Peripheral 
******************************************************************************/

/** @defgroup DMA
 *  @{
 */
 
#ifndef DMA_API_H
#define DMA_API_H

#ifdef __cplusplus
extern "C" {
#endif

enum device_names
{
	SMBUS_SLAVE=0,
	SMBUS_MASTER,
	SPI_FLASH,
	DEV_NAME_RSVD1,
	DEV_NAME_RSVD2,
	DEV_NAME_MAX
};

enum device_ids
{
	DEVICE_ID_SLAVE_1 = 0,
	DEVICE_ID_MASTER_1,
	DEVICE_ID_SLAVE_2,
	DEVICE_ID_MASTER_2,
	DEVICE_ID_SPI_FLASH
};

#define DMA_CHANNEL_MAX	(10U)

/******************************************************************************/
/** get_device_id function.
 * This function retrieves the device id based on device name and instance
 * @param device_name - The device name - smb slave / smb master / spi flash
 * @return the device id
*******************************************************************************/
uint8_t get_device_id(const uint8_t device_name, const uint8_t device_instance);

#ifdef __cplusplus
}
#endif

#endif /*DMA_API_H*/

/* end of dma_api.h */
/**   @}
 */
