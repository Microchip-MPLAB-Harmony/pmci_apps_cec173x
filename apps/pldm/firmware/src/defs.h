/*
 **********************************************************************************
* � 2013 Microchip Technology Inc. and its subsidiaries.
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
 **********************************************************************************
 *  defs.h
 *      This is the definition header file for generic usages
 **********************************************************************************
 *  #xx
 **********************************************************************************
 * $File: //depot_pcs/FWEng/solutions/Glacier_GEN3/maincodeline/Harmony/pmci_stack/pmci_apps_cec173x/apps/pldm/firmware/src/defs.h $
 */


/*********************************************************************************/
/** @defgroup defs defs
 *  @{
 */

/** @file defs.h
* \brief definition header file for generic usages
* \author App Firmware Team
* 
**********************************************************************************/
#ifndef DEFS_H
#define DEFS_H

/* bit operation MACRO, xvar could be byte, word or dword */
#define mSET_BIT(x, xvar)    ( xvar |= x )
#define mCLR_BIT(x, xvar)    ( xvar &= ~x )
#define mGET_BIT(x, xvar)     ( xvar & x )
#define mCLR_SRC_BIT(x, xvar)   ( xvar = x )
#define mTOGGLE_BIT(x, xvar)    {if(mGET_BIT(x, xvar)){mCLR_BIT(x, xvar);}else{mSET_BIT(x, xvar);}}

#endif /*DEFS_H*/

/**   @}
 */

