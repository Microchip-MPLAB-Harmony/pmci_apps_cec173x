/*****************************************************************************
* � 2020 Microchip Technology Inc. and its subsidiaries.
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
/** @file gpio_api.c
* \brief GPIO API Source file
* \author pramans
*
* This file implements the GPIO API functions
******************************************************************************/

/** @defgroup GPIO
 *  @{
 */

#include "common.h"
#include "peripheral/gpio/plib_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NUM_GPIO_PORTS      6U
#define MAX_NUM_GPIO        (NUM_GPIO_PORTS * 32U)

//
// Logical bit map representation of each ports
//
#define GPIO_BANK0_BITMAP (0x7FFFFFFFUL) /* 0000 - 0037 */
#define GPIO_BANK1_BITMAP (0x7FFFFFFFUL) /* 0040 - 0077 */
#define GPIO_BANK2_BITMAP (0x7FFFFFFFUL) /* 0100 - 0137 */
#define GPIO_BANK3_BITMAP (0x7FFFFFFFUL) /* 0140 - 0177 */
#define GPIO_BANK4_BITMAP (0x7FFFFFFFUL) /* 0200 - 0237 */
#define GPIO_BANK5_BITMAP (0x00FFFFFFUL) /* 0240 - 0277 */

//
// Logical bit map representation of each ports
//
#define GPIO_BANK0_BITMAP (0x7FFFFFFFUL) /* 0000 - 0037 */
#define GPIO_BANK1_BITMAP (0x7FFFFFFFUL) /* 0040 - 0077 */
#define GPIO_BANK2_BITMAP (0x7FFFFFFFUL) /* 0100 - 0137 */
#define GPIO_BANK3_BITMAP (0x7FFFFFFFUL) /* 0140 - 0177 */
#define GPIO_BANK4_BITMAP (0x7FFFFFFFUL) /* 0200 - 0237 */
#define GPIO_BANK5_BITMAP (0x00FFFFFFUL) /* 0240 - 0277 */

#define MAX_PIN					(GPIO_PIN_GPIO253 + 1U)

static uint8_t gpio_is_valid( GPIO_PIN pin );

// GPIO Port bitmap
static const uint32_t valid_ctrl_masks[NUM_GPIO_PORTS] = {
    (GPIO_BANK0_BITMAP),
    (GPIO_BANK1_BITMAP),
    (GPIO_BANK2_BITMAP),
    (GPIO_BANK3_BITMAP),
    (GPIO_BANK4_BITMAP),
    (GPIO_BANK5_BITMAP)
};

/* ------------------------------------------------------------------------------ */
/*                      Function to set gpio property                             */
/* ------------------------------------------------------------------------------ */
/**
 * gpio_property_set - updates the configuration of the specified gpio pin
 * at run time.
 * @param 0-based GPIO ID
 * @param property type that is to be changed
 * @param new value
 * @return uint8_t 0(success), 1(fail).
 */
uint8_t gpio_property_set( GPIO_PIN pin, GPIO_PROPERTY gpio_prop, const uint32_t new_prop_val )
{
    uint8_t retCode = 0u;
    if ( 1u == gpio_is_valid(pin) )
    {
        GPIO_PropertySet(pin, gpio_prop, new_prop_val);
        retCode = 0u;
    }
    else
    {
        retCode = 1u;
    }
    return retCode;
}

/* ------------------------------------------------------------------------------ */
/*                      Function to initialize gpio pin                           */
/* ------------------------------------------------------------------------------ */
/**
 * gpio_init - inititializes the specified gpio pin.
 * @param 0-based GPIO ID
 * @param input disable
 * @param control mux mode
 * @param pin polarity
 * @param direction of the pin
 * @param output buffer type
 * @param interrupt detection mode
 * @param power gate source
 * @param internal resistor mode
 * @return uint8_t 0(success), 1(fail).
 */
uint8_t gpio_init( GPIO_PIN pin, GPIO_INP_READ new_val, GPIO_FUNCTION new_mux, GPIO_POLARITY new_pol, GPIO_DIR new_dir, \
                   GPIO_OUTPUT_BUFFER_TYPE new_obuf, GPIO_INTDET_TYPE new_idet, GPIO_PWRGATE new_pwrg, GPIO_PULL_TYPE new_pud )
{
    uint8_t ret_init_sts = 0u;
    if ( 1u == gpio_is_valid(pin) )
    {
		(void)gpio_property_set(pin, GPIO_PROP_INP_EN_DIS, (uint32_t)new_val);
		(void)gpio_property_set(pin, GPIO_PROP_MUX_SEL, (uint32_t)new_mux);
		(void)gpio_property_set(pin, GPIO_PROP_POLARITY, (uint32_t)new_pol);
		(void)gpio_property_set(pin, GPIO_PROP_DIR, (uint32_t)new_dir);
		(void)gpio_property_set(pin, GPIO_PROP_OBUFF_TYPE, (uint32_t)new_obuf);
		(void)gpio_property_set(pin, GPIO_PROP_INT_DET, (uint32_t)new_idet);
		(void)gpio_property_set(pin, GPIO_PROP_PWR_GATE, (uint32_t)new_pwrg);
		(void)gpio_property_set(pin, GPIO_PROP_PU_PD, (uint32_t)new_pud);
		
        ret_init_sts = 0u;
    }
    else
    {
        ret_init_sts = 1u;
    }
	
    return ret_init_sts;
}

/* ------------------------------------------------------------------------------ */
/*                          Function to write to the pad                          */
/* ------------------------------------------------------------------------------ */
/**
 * gpio_output_set - writes the output value to the specified
 * pin depending upon the output source register.
 * @param 0-based GPIO ID
 * @param output source register
 * @param new output value
 * @return uint8_t 0(success), 1(fail).
 *
 */

uint8_t gpio_output_set( GPIO_PIN pin, GPIO_ALT_OUT out_src, const uint32_t gpio_state )
{
    uint8_t ret_op_set_sts = 0u;

    if ( 1u == gpio_is_valid(pin) )
    {
		GPIO_PinGroupOutputConfig( pin, out_src );

        if ( GPIO_ALT_OUT_DIS == out_src )
        {
			if(gpio_state > 0u) {
				GPIO_GroupPinSet(pin);
			} else {
				GPIO_GroupPinClear(pin);
			}
        }
        else
        {
			if(gpio_state > 0u) {
				GPIO_PinSet(pin);
			} else {
				GPIO_PinClear(pin);
			}
        }

        ret_op_set_sts = 0u;
    }
    else
    {
        ret_op_set_sts = 1u;
    }
    return ret_op_set_sts;
}

/* ------------------------------------------------------------------------------ */
/*               Function to read the pad input using the input register          */
/* ------------------------------------------------------------------------------ */
/**
 * gpio_input_get - reads the input from the specified gpio pin
 * using the input register.
 * @param 0-based GPIO ID
 * @return uint8_t current input value, 0xFF(fail).
 */
uint8_t gpio_input_get( GPIO_PIN pin )
{
    uint8_t ret_ip_get_sts = 0xFFu;
    if ( 1u == gpio_is_valid(pin) )
    {
        ret_ip_get_sts = GPIO_PinRead( pin );
    }
    else
    {
        ret_ip_get_sts = 0xFFu;
    }
    return ret_ip_get_sts;
}


/* ------------------------------------------------------------------------------ */
/*                      Function to configure gpio slew rate value                */
/* ------------------------------------------------------------------------------ */
/**
 * gpio_slewRate_set - sets the slew rate setting of the
 * specified gpio pin.
 * @param 0-based GPIO ID
 * @param new slew value
 * @return uint8_t 0(success), 1(fail).
 */
uint8_t gpio_slewRate_set( GPIO_PIN pin, GPIO_SLEW_RATE new_slew )
{
    uint8_t ret_slw_set_sts = 0u;
    if ( 1u == gpio_is_valid(pin) )
    {
		GPIO_PinSlewRateConfig(pin, new_slew);

        ret_slw_set_sts = 0u;
    }
    else
    {
        ret_slw_set_sts = 1u;
    }

    return ret_slw_set_sts;
}


/* ------------------------------------------------------------------------------ */
/*                  Function to configure gpio drive strength value               */
/* ------------------------------------------------------------------------------ */
/**
 * gpio_drvStr_set -sets the drive strength setting of the
 * specified gpio pin.
 * @param 0-based GPIO ID
 * @param new drive strength setting
 * @return uint8_t 0(success), 1(fail).
 */
uint8_t gpio_drvStr_set( GPIO_PIN pin, GPIO_DRV drv_str )
{
    uint8_t ret_drvstr_set_sts = 0u;
    if ( 1u == gpio_is_valid(pin) )
    {
		GPIO_DrvStrConfig(pin, drv_str);
		
        ret_drvstr_set_sts = 0u;
    }
    else
    {
        ret_drvstr_set_sts = 1u;
    }
    return ret_drvstr_set_sts;
}

/* ------------------------------------------------------------------------------ */
/*                  Function to configure a default output high gpio              */
/* ------------------------------------------------------------------------------ */
/**
 * gpio_init_default_output_high - For default output high gpio pin, it should be
 * configured as per below steps to avoid glitches
 * specified gpio pin.
 * @param GPIO PIN number
 * @return void.
 */
void gpio_init_default_output_high( GPIO_PIN pin)
{
    (void)gpio_init( pin, GPIO_INP_ENABLE, GPIO_FUNCTION_GPIO, GPIO_POLARITY_NON_INVERTED, \
               GPIO_DIR_INPUT, GPIO_OUTPUT_BUFFER_TYPE_PUSH_PULL, GPIO_INTDET_TYPE_DISABLED, GPIO_PWR_VTR,\
               GPIO_PULL_TYPE_NONE );

    (void)gpio_output_set( pin, GPIO_ALT_OUT_DIS, 1u );

    (void)gpio_property_set( pin, GPIO_PROP_DIR, (uint32_t)GPIO_DIR_OUTPUT );

}

/* ------------------------------------------------------------------------------ */
/*                  Function to check if the gpio pin is a valid one              */
/* ------------------------------------------------------------------------------ */
/**
 * gpio_is_valid - Checks if GPIO pin is implemented in this hardware.
 * @param 0-based GPIO ID
 * @return uint8_t 1(GPIO Pin implemented), 0(not implemented).
 */
static uint8_t gpio_is_valid( GPIO_PIN pin )
{
    uint8_t ret_valid_sts = 0u;
    uint16_t gp_bank = 0u;

    if ( pin < GPIO_PIN_NONE )
    {
        gp_bank = (uint16_t)pin >> 5U;

        if ((valid_ctrl_masks[gp_bank] & (1UL << ((uint32_t)pin & 0x001FUL))) != 0u)
        {
            ret_valid_sts = 1u;
        }
    }
    else
    {
        ret_valid_sts = 0u;
    }

    return ret_valid_sts;
}

#ifdef __cplusplus
}
#endif

/* end of gpio_api.c */
/**   @}
 */
