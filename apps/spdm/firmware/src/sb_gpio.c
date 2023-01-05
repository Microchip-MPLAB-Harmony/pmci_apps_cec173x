/*****************************************************************************
* © 2020 Microchip Technology Inc. and its subsidiaries.
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

/** @file sb_gpio.c
 * Secure Boot - GPIO
 */
/** @defgroup secure_boot 
 */

/* Includes */

#include <common.h>
#include "sb_gpio.h"


/******************************************************************************/
/** gpio_init_pp_output();
* Sets GPIO to push pull output 
* @param pin_id GPIO pin id
* @return None
*******************************************************************************/
void gpio_init_pp_output(GPIO_PIN pin_id)
{
    gpio_init( pin_id, GPIO_INP_DISABLE, GPIO_FUNCTION_GPIO, GPIO_POLARITY_NON_INVERTED, GPIO_DIR_OUTPUT, \
                   GPIO_OUTPUT_BUFFER_TYPE_PUSH_PULL, GPIO_INTDET_TYPE_DISABLED, GPIO_PWR_VTR, GPIO_PULL_TYPE_NONE );   
}

/******************************************************************************/
/** gpio_init_pp_input();
* Sets GPIO to push pull input configuration
* @param pin_id GPIO pin id
* @return None
*******************************************************************************/
void gpio_init_pp_input(GPIO_PIN pin_id)
{
    gpio_init( pin_id, GPIO_INP_ENABLE, GPIO_FUNCTION_GPIO, GPIO_POLARITY_NON_INVERTED, GPIO_DIR_INPUT, \
                   GPIO_OUTPUT_BUFFER_TYPE_PUSH_PULL, GPIO_INTDET_TYPE_DISABLED, GPIO_PWR_VTR, GPIO_PULL_TYPE_NONE );   
}

/******************************************************************************/
/** gpio_init_pp_output_low();
* Sets GPIO to push pull output low
* @param pin_id GPIO pin id
* @return None
*******************************************************************************/
void gpio_init_pp_output_low(GPIO_PIN pin_id)
{
    gpio_init_pp_input(pin_id);
    
    gpio_output_set( pin_id, GPIO_ALT_OUT_DIS, 0u );
    
    gpio_property_set( pin_id, GPIO_PROP_DIR, GPIO_DIR_OUTPUT );
}

/******************************************************************************/
/** gpio_init_pp_output_high();
* Sets GPIO to push pull output high
* @param pin_id GPIO pin id
* @return None
*******************************************************************************/
void gpio_init_pp_output_high(GPIO_PIN pin_id)
{

    gpio_init_pp_input(pin_id);
    
    gpio_output_set( pin_id, GPIO_ALT_OUT_DIS, 1u );
    
    gpio_property_set( pin_id, GPIO_PROP_DIR, GPIO_DIR_OUTPUT );
    
}