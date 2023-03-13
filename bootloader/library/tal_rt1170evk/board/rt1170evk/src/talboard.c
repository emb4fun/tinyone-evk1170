/**************************************************************************
*  This file is part of the TAL project (Tiny Abstraction Layer)
*
*  Copyright (c) 2022 by Michael Fischer (www.emb4fun.de).
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without 
*  modification, are permitted provided that the following conditions 
*  are met:
*  
*  1. Redistributions of source code must retain the above copyright 
*     notice, this list of conditions and the following disclaimer.
*
*  2. Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in the 
*     documentation and/or other materials provided with the distribution.
*
*  3. Neither the name of the author nor the names of its contributors may 
*     be used to endorse or promote products derived from this software 
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL 
*  THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS 
*  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED 
*  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF 
*  THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
*  SUCH DAMAGE.
**************************************************************************/
#if defined(USE_BOARD_RT1170EVK)
#define __TALBOARD_C__

/*=======================================================================*/
/*  Include                                                              */
/*=======================================================================*/
#include <string.h>
#include "tal.h"

#include "fsl_iomuxc.h"
#include "fsl_silicon_id.h"

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

/*=======================================================================*/
/*  Definition of all global Data                                        */
/*=======================================================================*/

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

/*=======================================================================*/
/*  All code exported                                                    */
/*=======================================================================*/

#if defined(MEMORY_INIT)
/*************************************************************************/
/*  MemoryInit                                                           */
/*                                                                       */
/*  If MEMORY_INIT is defined, the MemoryInit() function will be called. */
/*  By default  MemoryInit() is called after SystemInit() to enable an   */
/*  external memory controller.                                          */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void MemoryInit (void)
{
   /*
    * SDRAM is still by evkmimxrt1020_sdram_init.jlinkscript
    */
   
} /* MemoryInit */
#endif

/*************************************************************************/
/*  tal_BoardEnableCOMx                                                  */
/*                                                                       */
/*  Board and hardware depending functionality to enable the COM port.   */
/*  If this port is not supported by the board, return an error.         */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
TAL_RESULT tal_BoardEnableCOM1 (void)
{
   CLOCK_EnableClock(kCLOCK_Iomuxc);      /* LPCG on: LPCG is ON. */

   IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_24_LPUART1_TXD,      /* GPIO_AD_24 is configured as LPUART1_TXD */
      0U);                                /* Software Input On Field: Input Path is determined by functionality */
   IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_25_LPUART1_RXD,      /* GPIO_AD_25 is configured as LPUART1_RXD */
      1U);                                /* Software Input On Field: Force input path of pad GPIO_AD_25 */

   IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AD_24_LPUART1_TXD,      /* GPIO_AD_24 PAD functional properties : */
      0x02U);                             /* Slew Rate Field: Slow Slew Rate
                                             Drive Strength Field: high drive strength
                                             Pull / Keep Select Field: Pull Disable, Highz
                                             Pull Up / Down Config. Field: Weak pull down
                                             Open Drain Field: Disabled
                                             Domain write protection: Both cores are allowed
                                             Domain write protection lock: Neither of DWP bits is locked */
   IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AD_25_LPUART1_RXD,      /* GPIO_AD_25 PAD functional properties : */
      0x02U);                             /* Slew Rate Field: Slow Slew Rate
                                             Drive Strength Field: high drive strength
                                             Pull / Keep Select Field: Pull Disable, Highz
                                             Pull Up / Down Config. Field: Weak pull down
                                             Open Drain Field: Disabled
                                             Domain write protection: Both cores are allowed
                                             Domain write protection lock: Neither of DWP bits is locked */

   return(TAL_OK);
} /* tal_BoardEnableCOM1 */

TAL_RESULT tal_BoardEnableCOM2 (void)
{
   return(TAL_ERR_COM_PORT_NOHW);
} /* tal_BoardEnableCOM2 */

TAL_RESULT tal_BoardEnableCOM3 (void)
{
   return(TAL_ERR_COM_PORT_NOHW);
} /* tal_BoardEnableCOM3 */

TAL_RESULT tal_BoardEnableCOM4 (void)
{
   return(TAL_ERR_COM_PORT_NOHW);
} /* tal_BoardEnableCOM4 */

TAL_RESULT tal_BoardEnableCOM5 (void)
{
   return(TAL_ERR_COM_PORT_NOHW);
} /* tal_BoardEnableCOM5 */

TAL_RESULT tal_BoardEnableCOM6 (void)
{
   return(TAL_ERR_COM_PORT_NOHW);
} /* tal_BoardEnableCOM6 */

TAL_RESULT tal_BoardEnableCOM7 (void)
{
   return(TAL_ERR_COM_PORT_NOHW);
} /* tal_BoardEnableCOM7 */

TAL_RESULT tal_BoardEnableCOM8 (void)
{
   return(TAL_ERR_COM_PORT_NOHW);
} /* tal_BoardEnableCOM8 */

/*************************************************************************/
/*  tal_BoardEnableCANx                                                  */
/*                                                                       */
/*  Board and hardware depending functionality to enable the CAN port.   */
/*  If this port is not supported by the board, return an error.         */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
TAL_RESULT tal_BoardEnableCAN1 (void)
{
   return(TAL_ERR_CAN_PORT_NOHW);
} /* tal_BoardEnableCAN1 */

TAL_RESULT tal_BoardEnableCAN2 (void)
{
   return(TAL_ERR_CAN_PORT_NOHW);
} /* tal_BoardEnableCAN2 */

/*************************************************************************/
/*  tal_BoardGetMACAddress                                               */
/*                                                                       */
/*  Retrieve the MAC address of the board.                               */
/*  In case of an error, a default address will be used.                 */
/*                                                                       */
/*  In    : iface, pAddress                                              */
/*  Out   : pAddress                                                     */
/*  Return: TAL_OK / TAL_ERROR                                           */
/*************************************************************************/
TAL_RESULT tal_BoardGetMACAddress (int iface, uint8_t *pAddress)
{
   TAL_RESULT  Error = TAL_ERROR;
   status_t    result;
   uint8_t     siliconId[SILICONID_MAX_LENGTH];
   uint32_t    idLen;
   
   /* Inspired by SILICONID_ConvertToMacAddr */

   result = SILICONID_GetID(&siliconId[0], &idLen);
   if ((kStatus_Success == result) && (idLen >= 3U))
   {
      if      (0 == iface)
      {
         Error = TAL_OK;
         
         pAddress[0] = 0x54;
         pAddress[1] = 0x27;
         pAddress[2] = 0x8d;
         pAddress[3] = siliconId[0];
         pAddress[4] = siliconId[1];
         pAddress[5] = siliconId[2];
      }  
      else if (1 == iface)
      {
         Error = TAL_OK;
         
         pAddress[0] = 0x54;
         pAddress[1] = 0x27;
         pAddress[2] = 0x8d;
         pAddress[3] = siliconId[0];
         pAddress[4] = siliconId[1];
         pAddress[5] = siliconId[2]+1;
      }  
   }      
   
   return(Error);
} /* tal_BoardGetMACAddress */

/*************************************************************************/
/*  tal_BoardRTCSetTM                                                    */
/*                                                                       */
/*  Set the onboard RTC time by TM                                       */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_BoardRTCSetTM (struct tm *pTM)
{
   (void)pTM;
} /* tal_BoardRTCSetTM */

/*************************************************************************/
/*  tal_BoardRTCSetUnixtime                                              */
/*                                                                       */
/*  Set the onboard RTC time by Unixtime                                 */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_BoardRTCSetUnixtime (uint32_t Unixtime)
{
   (void)Unixtime;
} /* tal_BoardRTCSetUnixtime */

/*************************************************************************/
/*  tal_BoardRTC2System                                                  */
/*                                                                       */
/*  Set the system time from the RTC.                                    */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_BoardRTC2System (void)
{
} /* tal_BoardRTC2System */

#endif /* USE_BOARD_RT1170EVK */

/*** EOF ***/
