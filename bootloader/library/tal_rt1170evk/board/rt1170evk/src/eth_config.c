/**************************************************************************
*  This file is part of the TAL project (Tiny Abstraction Layer)
*
*  Copyright (c) 2020-2022 by Michael Fischer (www.emb4fun.de).
*  All rights reserved.
*
*  Based on an example from ST. Therefore partial copyright:
*  Copyright (c) 2016 STMicroelectronics
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
#define __ETH_CONFIG_C__

/*
 * Copyright 2017-2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*=======================================================================*/
/*  Include                                                              */
/*=======================================================================*/
#include <stdint.h>
#include "tal.h"
#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "fsl_gpio.h"
#include "fsl_phy.h"
#include "mdio\enet\fsl_enet_mdio.h"
#include "device\phyksz8081\fsl_phyksz8081.h"
#include "device\phyrtl8211f\fsl_phyrtl8211f.h"

#if !defined(FSL_FEATURE_PHYKSZ8081_USE_RMII50M_MODE)
   Error: FSL_FEATURE_PHYKSZ8081_USE_RMII50M_MODE must be definedby the preprocessor;
#endif   

/*=======================================================================*/
/*  Extern                                                               */
/*=======================================================================*/

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

static struct _phy_handle  phyHandle0;
static struct _mdio_handle mdioHandle0;

static struct _phy_handle  phyHandle1;
static struct _mdio_handle mdioHandle1;

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

/*************************************************************************/
/*  init0_module_clock                                                   */
/*                                                                       */
/*  Enable PLL output for ENET.                                          */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void init0_module_clock (void)
{
   /* 
    * BOARD_InitModuleClock 
    */ 
   const clock_sys_pll1_config_t sysPll1Config = {
      .pllDiv2En = true,
   };
   CLOCK_InitSysPll1(&sysPll1Config);

   clock_root_config_t rootCfg = {.mux = 4, .div = 10}; /* Generate 50M root clock. */
   
   CLOCK_SetRootClock(kCLOCK_Root_Enet1, &rootCfg);
   
   /*
    * IOMUXC_SelectENETClock
    */
   IOMUXC_GPR->GPR4 |= IOMUXC_GPR_GPR4_ENET_REF_CLK_DIR_MASK; /* 50M ENET_REF_CLOCK output to PHY and ENET module. */

} /* init0_module_clock */

/*************************************************************************/
/*  init1_module_clock                                                   */
/*                                                                       */
/*  Enable PLL output for ENET_1G.                                       */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void init1_module_clock (void)
{
   /* 
    * BOARD_InitModuleClock 
    */ 
   const clock_sys_pll1_config_t sysPll1Config = {
      .pllDiv2En = true,
   };
   CLOCK_InitSysPll1(&sysPll1Config);

   clock_root_config_t rootCfg = {.mux = 4, .div = 4}; /* Generate 125M root clock. */
   CLOCK_SetRootClock(kCLOCK_Root_Enet2, &rootCfg);
   
   /*
    * IOMUXC_SelectENETClock
    */
   IOMUXC_GPR->GPR5 |= IOMUXC_GPR_GPR5_ENET1G_RGMII_EN_MASK; /* bit1:iomuxc_gpr_enet_clk_dir
                                                                bit0:GPR_ENET_TX_CLK_SEL(internal or OSC) */
   
} /* init1_module_clock */

/*************************************************************************/
/*  enet0_pin_mux                                                        */
/*                                                                       */
/*  Configures pin routing and optionally pin electrical features.       */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void enet0_pin_mux (void)
{
   CLOCK_EnableClock(kCLOCK_Iomuxc);               /* LPCG on: LPCG is ON. */
   CLOCK_EnableClock(kCLOCK_Iomuxc_Lpsr);          /* LPCG on: LPCG is ON. */

   IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_12_GPIO9_IO11,                /* GPIO_AD_12 is configured as GPIO9_IO11 */
      0U);                                         /* Software Input On Field: Input Path is determined by functionality */
      
   IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_32_ENET_MDC,                  /* GPIO_AD_32 is configured as ENET_MDC */
      0U);                                         /* Software Input On Field: Input Path is determined by functionality */
   IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_33_ENET_MDIO,                 /* GPIO_AD_33 is configured as ENET_MDIO */
      0U);                                         /* Software Input On Field: Input Path is determined by functionality */
   IOMUXC_SetPinMux(
      IOMUXC_GPIO_DISP_B2_02_ENET_TX_DATA00,       /* GPIO_DISP_B2_02 is configured as ENET_TX_DATA00 */
      0U);                                         /* Software Input On Field: Input Path is determined by functionality */
   IOMUXC_SetPinMux(
      IOMUXC_GPIO_DISP_B2_03_ENET_TX_DATA01,       /* GPIO_DISP_B2_03 is configured as ENET_TX_DATA01 */
      0U);                                         /* Software Input On Field: Input Path is determined by functionality */
   IOMUXC_SetPinMux(
      IOMUXC_GPIO_DISP_B2_04_ENET_TX_EN,           /* GPIO_DISP_B2_04 is configured as ENET_TX_EN */
      0U);                                         /* Software Input On Field: Input Path is determined by functionality */
   IOMUXC_SetPinMux(
      IOMUXC_GPIO_DISP_B2_05_ENET_REF_CLK,         /* GPIO_DISP_B2_05 is configured as ENET_REF_CLK */
      1U);                                         /* Software Input On Field: Force input path of pad GPIO_DISP_B2_05 */
   IOMUXC_SetPinMux(
      IOMUXC_GPIO_DISP_B2_06_ENET_RX_DATA00,       /* GPIO_DISP_B2_06 is configured as ENET_RX_DATA00 */
      0U);                                         /* Software Input On Field: Input Path is determined by functionality */
   IOMUXC_SetPinMux(
      IOMUXC_GPIO_DISP_B2_07_ENET_RX_DATA01,       /* GPIO_DISP_B2_07 is configured as ENET_RX_DATA01 */
      0U);                                         /* Software Input On Field: Input Path is determined by functionality */
   IOMUXC_SetPinMux(
      IOMUXC_GPIO_DISP_B2_08_ENET_RX_EN,           /* GPIO_DISP_B2_08 is configured as ENET_RX_EN */
      0U);                                         /* Software Input On Field: Input Path is determined by functionality */
   IOMUXC_SetPinMux(
      IOMUXC_GPIO_DISP_B2_09_ENET_RX_ER,           /* GPIO_DISP_B2_09 is configured as ENET_RX_ER */
      0U);                                         /* Software Input On Field: Input Path is determined by functionality */
      
   IOMUXC_GPR->GPR4 = ((IOMUXC_GPR->GPR4 &
       (~(IOMUXC_GPR_GPR4_ENET_REF_CLK_DIR_MASK))) /* Mask bits to zero which are setting */
         | IOMUXC_GPR_GPR4_ENET_REF_CLK_DIR(0x01U) /* ENET_REF_CLK direction control: 0x01U */
       );
       
   IOMUXC_SetPinMux(
      IOMUXC_GPIO_LPSR_12_GPIO12_IO12,             /* GPIO_LPSR_12 is configured as GPIO12_IO12 */
      0U);                                         /* Software Input On Field: Input Path is determined by functionality */
   IOMUXC_SetPinConfig(
      IOMUXC_GPIO_DISP_B2_05_ENET_REF_CLK,         /* GPIO_DISP_B2_05 PAD functional properties : */
      0x03U);                                      /* Slew Rate Field: Fast Slew Rate
                                                      Drive Strength Field: high drive strength
                                                      Pull / Keep Select Field: Pull Disable, Highz
                                                      Pull Up / Down Config. Field: Weak pull down
                                                      Open Drain Field: Disabled
                                                      Domain write protection: Both cores are allowed
                                                      Domain write protection lock: Neither of DWP bits is locked */

} /* enet0_pin_mux */

static void enet1_pin_mux (void)
{
   CLOCK_EnableClock(kCLOCK_Iomuxc);               /* LPCG on: LPCG is ON. */

   IOMUXC_SetPinMux(
      IOMUXC_GPIO_DISP_B1_00_ENET_1G_RX_EN,        /* GPIO_DISP_B1_00 is configured as ENET_1G_RX_EN */
      0U);                                         /* Software Input On Field: Input Path is determined by functionality */
   IOMUXC_SetPinMux(
      IOMUXC_GPIO_DISP_B1_01_ENET_1G_RX_CLK,       /* GPIO_DISP_B1_01 is configured as ENET_1G_RX_CLK */
      0U);                                         /* Software Input On Field: Input Path is determined by functionality */
   IOMUXC_SetPinMux(
      IOMUXC_GPIO_DISP_B1_02_ENET_1G_RX_DATA00,    /* GPIO_DISP_B1_02 is configured as ENET_1G_RX_DATA00 */
      0U);                                         /* Software Input On Field: Input Path is determined by functionality */
   IOMUXC_SetPinMux(
      IOMUXC_GPIO_DISP_B1_03_ENET_1G_RX_DATA01,    /* GPIO_DISP_B1_03 is configured as ENET_1G_RX_DATA01 */
      0U);                                         /* Software Input On Field: Input Path is determined by functionality */
   IOMUXC_SetPinMux(
      IOMUXC_GPIO_DISP_B1_04_ENET_1G_RX_DATA02,    /* GPIO_DISP_B1_04 is configured as ENET_1G_RX_DATA02 */
      0U);                                         /* Software Input On Field: Input Path is determined by functionality */
   IOMUXC_SetPinMux(
      IOMUXC_GPIO_DISP_B1_05_ENET_1G_RX_DATA03,    /* GPIO_DISP_B1_05 is configured as ENET_1G_RX_DATA03 */
      0U);                                         /* Software Input On Field: Input Path is determined by functionality */
   IOMUXC_SetPinMux(
      IOMUXC_GPIO_DISP_B1_06_ENET_1G_TX_DATA03,    /* GPIO_DISP_B1_06 is configured as ENET_1G_TX_DATA03 */
      0U);                                         /* Software Input On Field: Input Path is determined by functionality */
   IOMUXC_SetPinMux(
      IOMUXC_GPIO_DISP_B1_07_ENET_1G_TX_DATA02,    /* GPIO_DISP_B1_07 is configured as ENET_1G_TX_DATA02 */
      0U);                                         /* Software Input On Field: Input Path is determined by functionality */
   IOMUXC_SetPinMux(
      IOMUXC_GPIO_DISP_B1_08_ENET_1G_TX_DATA01,    /* GPIO_DISP_B1_08 is configured as ENET_1G_TX_DATA01 */
      0U);                                         /* Software Input On Field: Input Path is determined by functionality */
   IOMUXC_SetPinMux(
      IOMUXC_GPIO_DISP_B1_09_ENET_1G_TX_DATA00,    /* GPIO_DISP_B1_09 is configured as ENET_1G_TX_DATA00 */
      0U);                                         /* Software Input On Field: Input Path is determined by functionality */
   IOMUXC_SetPinMux(
      IOMUXC_GPIO_DISP_B1_10_ENET_1G_TX_EN,        /* GPIO_DISP_B1_10 is configured as ENET_1G_TX_EN */
      0U);                                         /* Software Input On Field: Input Path is determined by functionality */
   IOMUXC_SetPinMux(
      IOMUXC_GPIO_DISP_B1_11_ENET_1G_TX_CLK_IO,    /* GPIO_DISP_B1_11 is configured as ENET_1G_TX_CLK_IO */
      0U);                                         /* Software Input On Field: Input Path is determined by functionality */
   IOMUXC_SetPinMux(
      IOMUXC_GPIO_DISP_B2_13_GPIO11_IO14,          /* GPIO_DISP_B2_13 is configured as GPIO11_IO14 */
      0U);                                         /* Software Input On Field: Input Path is determined by functionality */
   IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_B2_19_ENET_1G_MDC,           /* GPIO_EMC_B2_19 is configured as ENET_1G_MDC */
      0U);                                         /* Software Input On Field: Input Path is determined by functionality */
   IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_B2_20_ENET_1G_MDIO,          /* GPIO_EMC_B2_20 is configured as ENET_1G_MDIO */
      0U);                                         /* Software Input On Field: Input Path is determined by functionality */

} /* enet1_pin_mux */

/*=======================================================================*/
/*  All code exported                                                    */
/*=======================================================================*/

/*************************************************************************/
/*  BoardETHConfig                                                       */
/*                                                                       */
/*  Configures pin routing and optionally pin electrical features.       */
/*                                                                       */
/*  In    : iface                                                        */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void BoardETHConfig (int iface)
{
   gpio_pin_config_t gpio_config = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};

   if      (0 == iface)
   {
      init0_module_clock();

      enet0_pin_mux();
   
      GPIO_PinInit(GPIO9,  11, &gpio_config);
      GPIO_PinInit(GPIO12, 12, &gpio_config);
      /* pull up the ENET_INT before RESET. */
      GPIO_WritePinOutput(GPIO9,  11, 1);
      GPIO_WritePinOutput(GPIO12, 12, 0);
      OS_TimeDly(100);
      GPIO_WritePinOutput(GPIO12, 12, 1);   
   }
   else if (1 == iface)
   {
      init1_module_clock();

      enet1_pin_mux();

      GPIO_PinInit(GPIO11, 14, &gpio_config);
   
      GPIO_WritePinOutput(GPIO11, 14, 0);
      OS_TimeDly(100);
      GPIO_WritePinOutput(GPIO11, 14, 1);   
   }      

} /* BoardETHConfig */

/*************************************************************************/
/*  BoardGetPhyHandle                                                    */
/*                                                                       */
/*  Return PHY handle depending of the iface.                            */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
struct _phy_handle *BoardGetPhyHandle (int iface)
{
   struct _phy_handle *pHandle = NULL;
   
   if (0 == iface)
   {
      mdioHandle0.ops                  = &enet_ops;
      mdioHandle0.resource.base        = ENET;
      mdioHandle0.resource.csrClock_Hz = CLOCK_GetRootClockFreq(kCLOCK_Root_Bus);
   
      phyHandle0.phyAddr    = 2;
      phyHandle0.mdioHandle = &mdioHandle0;
      phyHandle0.ops        = &phyksz8081_ops;
   
      pHandle = &phyHandle0;
   }
   else if (1 == iface)
   {
      mdioHandle1.ops                  = &enet_ops;
      mdioHandle1.resource.base        = ENET_1G;
      mdioHandle1.resource.csrClock_Hz = CLOCK_GetRootClockFreq(kCLOCK_Root_Bus);
   
      phyHandle1.phyAddr    = 1;
      phyHandle1.mdioHandle = &mdioHandle1;
      phyHandle1.ops        = &phyrtl8211f_ops;
   
      pHandle = &phyHandle1;
   }      
   
   return(pHandle);
} /* BoardGetPhyHandle */

/*** EOF ***/
