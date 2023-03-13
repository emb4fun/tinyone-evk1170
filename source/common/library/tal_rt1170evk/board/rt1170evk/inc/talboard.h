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
#if !defined(__TALBOARD_H__) && defined(USE_BOARD_RT1170EVK)
#define __TALBOARD_H__

/**************************************************************************
*  Includes
**************************************************************************/
#include <time.h>
#include "project.h"
#include "taltypes.h"
#include "heap_conf.h"

#include "imxrt_cpu.h"

/**************************************************************************
*  Global Definitions
**************************************************************************/

/*
 * Defines for board and cpu name
 */
#define TAL_BOARD "i.MX RT1170-EVK"
#define TAL_CPU   "RT1176 (m7)"

/*
 * With 4 (__NVIC_PRIO_BITS) bits of interrupt priority 16 levels are
 * available. Priority 0 is the highest one, 15 the lowest.
 */

#ifndef SYSTICK_PRIO
#define SYSTICK_PRIO       0
#endif 

#ifndef UART1_PRIO
#define UART1_PRIO         7
#endif


/*
 * The Embedded Artists LPC4088 Developer's Kit support 2 LEDs
 */
typedef enum _tal_led_channel_
{
   TAL_LED_CHANNEL_1 = 0,
   TAL_LED_CHANNEL_2,
   
   /* TAL_LED_CHANNEL_MAX must be the last one */
   TAL_LED_CHANNEL_MAX
} TAL_LED_CHANNEL;

#define LED_GREEN TAL_LED_CHANNEL_1
#define LED_RED   TAL_LED_CHANNEL_2


/*
 * Max supported ethernet interfaces
 */
#define ETH_MAX_IFACE   2


/*
 * COM port used for the terminal
 */
#if !defined(TERM_COM_PORT)
#define TERM_COM_PORT   TAL_COM_PORT_1
#endif

/**************************************************************************
*  Macro Definitions
**************************************************************************/

/**************************************************************************
*  Functions Definitions
**************************************************************************/

void       tal_BoardMemInit (void);

TAL_RESULT tal_BoardEnableCOM1 (void);
TAL_RESULT tal_BoardEnableCOM2 (void);
TAL_RESULT tal_BoardEnableCOM3 (void);
TAL_RESULT tal_BoardEnableCOM4 (void);
TAL_RESULT tal_BoardEnableCOM5 (void);
TAL_RESULT tal_BoardEnableCOM6 (void);
TAL_RESULT tal_BoardEnableCOM7 (void);
TAL_RESULT tal_BoardEnableCOM8 (void);

TAL_RESULT tal_BoardEnableCAN1 (void);
TAL_RESULT tal_BoardEnableCAN2 (void);

TAL_RESULT tal_BoardGetMACAddress (int iface, uint8_t *pAddress);

void       tal_BoardRTCSetTM (struct tm *pTM);
void       tal_BoardRTCSetUnixtime (uint32_t Unixtime);
void       tal_BoardRTC2System (void);

#endif /* !__TALBOARD_H__ */

/*** EOF ***/
