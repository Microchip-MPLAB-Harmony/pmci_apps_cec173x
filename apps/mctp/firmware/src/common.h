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

#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>
#include <stddef.h>
#include "definitions.h"
#include "defs.h"

#include "interrupt/interrupt_api.h"
#include "gpio/gpio_api.h"
#include "dma/dma_api.h"
#include "peripheral/i2c/plib_i2c_smbus_lowlevel_intf.h"

#define BIT_n_MASK(n)	(1ul << (n))
#define BIT_0_MASK    (1<<0)
#define BIT_1_MASK    (1<<1)
#define BIT_2_MASK    (1<<2)
#define BIT_3_MASK    (1<<3)
#define BIT_4_MASK    (1<<4)
#define BIT_5_MASK    (1<<5)
#define BIT_6_MASK    (1<<6)
#define BIT_7_MASK    (1<<7)
#define BIT_8_MASK    ((UINT16)1<<8)
#define BIT_9_MASK    ((UINT16)1<<9)
#define BIT_10_MASK   ((UINT16)1<<10)
#define BIT_11_MASK   ((UINT16)1<<11)
#define BIT_12_MASK   ((UINT16)1<<12)
#define BIT_13_MASK   ((UINT16)1<<13)
#define BIT_14_MASK   ((UINT16)1<<14)
#define BIT_15_MASK   ((UINT16)1<<15)
#define BIT_16_MASK     ((UINT32)1<<16)
#define BIT_17_MASK     ((UINT32)1<<17)
#define BIT_18_MASK     ((UINT32)1<<18)
#define BIT_19_MASK     ((UINT32)1<<19)
#define BIT_20_MASK     ((UINT32)1<<20)
#define BIT_21_MASK     ((UINT32)1<<21)
#define BIT_22_MASK     ((UINT32)1<<22)
#define BIT_23_MASK     ((UINT32)1<<23)
#define BIT_24_MASK     ((UINT32)1<<24)
#define BIT_25_MASK     ((UINT32)1<<25)
#define BIT_26_MASK     ((UINT32)1<<26)
#define BIT_27_MASK     ((UINT32)1<<27)
#define BIT_28_MASK     ((UINT32)1<<28)
#define BIT_29_MASK     ((UINT32)1<<29)
#define BIT_30_MASK     ((UINT32)1<<30)
#define BIT_31_MASK     ((UINT32)1<<31)

#define BIT(x) ( 1u << x )

#define TRUE        (1u)
#define FALSE       (0u)

#define NULLVOIDPTR                 (void *)(0)

#define TICK_PERIOD     10

/* Macros useful scheduling & calculating timing loops */
#define ms_to_ticks(x)  (x/TICK_PERIOD)
#define second_to_ticks(x)  (x*1000/TICK_PERIOD)

#define DISABLE_INTERRUPTS() __disable_irq()
#define ENABLE_INTERRUPTS() __enable_irq()

#define MAX_IRQn (PERIPH_MAX_IRQn + 1)

#define trace0(nbr, cat, lvl, str)                        { }
#define trace1(nbr, cat, lvl, str, p1)                    { }
#define trace2(nbr, cat, lvl, str, p1, p2)                { }

/* Short form for Standard Data Types */
typedef unsigned char           UINT8;
typedef unsigned short          UINT16;
typedef unsigned long           UINT32;

typedef volatile unsigned char  REG8;

typedef unsigned char           BYTE;
typedef unsigned short          WORD;
typedef unsigned long           DWORD;

typedef unsigned char           UCHAR;
typedef unsigned short          USHORT;
typedef unsigned long           ULONG;

typedef unsigned char           BOOL;

typedef unsigned int            UINT;

/* signed types */
typedef signed char             INT8;
typedef signed short            INT16;
typedef signed long             INT32;

typedef void                    VOID;

typedef volatile unsigned char      VUINT8;
typedef volatile unsigned short int VUINT16;
typedef volatile unsigned long int  VUINT32;

#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif

#endif /* COMMON_H */
