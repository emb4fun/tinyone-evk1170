/**************************************************************************
*  Copyright (c) 2020 by Michael Fischer (www.emb4fun.de).
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
*
***************************************************************************
*  History:
*
*  12.03.2023  mifi  First Version for a NXP "i.MX RT1060 Evaluation Kit".
**************************************************************************/
#define __MAIN_C__

/*=======================================================================*/
/*  Includes                                                             */
/*=======================================================================*/
#include <stdint.h>
#include <stdio.h>
#include "tal.h"
#include "terminal.h"
#include "ff.h"
#include "diskio.h"
#include "adler32.h"
#include "xbin.h"
#include "xmempool.h"

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

//#define MIN(x, y) (((x) < (y)) ? (x) : (y))

#define LOGICAL_DRIVE         "0:/"
#define BOOT_NAME             "firmware.bin"

#define DATA_BUF_SIZE         (4*1024)

#define FIRMWARE_STAR_ADDR    0x80000000

/*=======================================================================*/
/*  Definition of all extern Data                                        */
/*=======================================================================*/

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

/*
 * Some TASK variables like stack and task control block.
 */
static OS_STACK (StartStack,  TASK_START_STK_SIZE);
static OS_TCB   TCBStartTask;

static FATFS         FSObject;
static XBIN_HEADER   Xbin;
static FIL          hFile;

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

/*************************************************************************/
/*  Reboot                                                               */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void Reboot (void)
{
   uint8_t bLoopCnt;

   for (bLoopCnt = 0; bLoopCnt < 5; bLoopCnt++)
   {
      tal_LEDSet(TAL_LED_CHANNEL_1);
      OS_TimeDly(500);

      tal_LEDClear(TAL_LED_CHANNEL_1);
      OS_TimeDly(500);
   }

   tal_CPUReboot();

} /* Reboot */

/*************************************************************************/
/*  Name  : DisableAllInterrupts                                         */
/*                                                                       */
/*  Disable all interrupts used before.                                  */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void DisableAllInterrupts (void)
{
   /* Disable SysTick */
   SysTick->CTRL = 0;

   NVIC_DisableIRQ(GPIO2_Combined_16_31_IRQn);
   NVIC_DisableIRQ(USDHC1_IRQn);
   NVIC_DisableIRQ(LPUART1_IRQn);

} /* DisableAllInterrupts */

/*************************************************************************/
/*  Name  : StartFirmware                                                */
/*                                                                       */
/*  Start the new firmware.                                              */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void StartFirmware (void)
{
   static void      (*StartFirm)(void);
   volatile uint32_t *pStack = (volatile uint32_t*)(FIRMWARE_STAR_ADDR + 0);
   volatile uint32_t *pReset = (volatile uint32_t*)(FIRMWARE_STAR_ADDR + 4);

   TAL_CPU_DISABLE_ALL_INTS();

   DisableAllInterrupts();

   StartFirm = (void (*)(void))*pReset;;

   /* Initialize new stack pointer */
   __set_MSP(*pStack);

   /* Set CONTROL and PRIMASK to "Reset" state */
   __set_CONTROL(0);
   __set_PRIMASK(0);

   /* Thanks for the tip with the cache */
   SCB_InvalidateICache();
   SCB_CleanInvalidateDCache();

   /* Start new firmware*/
   SCB->VTOR = FIRMWARE_STAR_ADDR;
   StartFirm();

   TAL_CPU_ENABLE_ALL_INTS();

} /* StartFirmware */

/*************************************************************************/
/*  ImageCopy                                                            */
/*                                                                       */
/*  Copies firmware image from non-volatile memory to RAM.               */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void ImageCopy (void)
{
   FRESULT       res;
   uint32_t     dBytesRead;
   uint8_t     *pDestAddr;
   XBIN_HEADER *pHeader = &Xbin;
   uint32_t     dCRC32;
   uint32_t     dDataTotalSize;
   uint32_t     dDataRead;

   pDestAddr = (uint8_t*)FIRMWARE_STAR_ADDR;

   /*
    * Open the file for reading
    */
   res = f_open(&hFile, BOOT_NAME, FA_READ);

   /*
    * If there was some problem opening the file, then return an error.
    */
   if (res != FR_OK)
   {
      term_printf("Unable to open \"%s\"\r\n", BOOT_NAME);
      Reboot();
   }
   else
   {
      term_printf("Copying firmware image from SD card to RAM\r\n");
   }

   /*
    * Read anc check XBIN header first
    */
   res = f_read(&hFile, &Xbin, sizeof(XBIN_HEADER), &dBytesRead);
   if (FR_OK == res)
   {
      res = FR_INT_ERR;

      /* Check XBIN header */
      if ( (XBIN_HEADER_MAGIC_1 == pHeader->dMagic1)      &&
           (XBIN_HEADER_MAGIC_2 == pHeader->dMagic2)      &&
           (XBIN_HEADER_SIZEVER == pHeader->dSizeVersion) )
      {
         /* Check CRC32 of the header */
         dCRC32 = adler32(ADLER_START_VALUE, (uint8_t*)pHeader, sizeof(XBIN_HEADER) - XBIN_SIZE_OF_CRC32);
         if (dCRC32 == pHeader->dHeaderCRC32)
         {
            res = FR_OK;
         }
      }
   }

   if (res != FR_OK)
   {
      f_close(&hFile);

      term_printf("Error CRC header\r\n");
      Reboot();
   }

   /*
    * Read data from the image and store it into RAM
    */
   dDataTotalSize = pHeader->dDataTotalSize;
   while (dDataTotalSize != 0)
   {
      dDataRead = MIN(DATA_BUF_SIZE, dDataTotalSize);
      res = f_read(&hFile, pDestAddr, dDataRead, &dBytesRead);
      if (res != FR_OK)
      {
         term_printf("Error read\r\n");
         Reboot();
      }

      if (dBytesRead != dDataRead)
      {
         term_printf("Error read\r\n");
         Reboot();
      }

      dDataTotalSize -= dBytesRead;
      pDestAddr      += dBytesRead;
   }

   f_close(&hFile);

   /*
    * Check image
    */
   dCRC32 = adler32(ADLER_START_VALUE, (uint8_t*)FIRMWARE_STAR_ADDR, pHeader->dDataTotalSize);
   if (dCRC32 != pHeader->dDataCRC32)
   {
      term_printf("Error CRC image\r\n");
      Reboot();
   }

} /* ImageCopy */

/*************************************************************************/
/*  OutputBootMessage                                                    */
/*                                                                       */
/*  Output boot message.                                                 */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void OutputBootMessage (void)
{
   const char ResetScreen[] = { 0x1B, 'c', 0 };

   term_printf("%s", ResetScreen);
   OS_TimeDly(50);

   term_printf("\r\n");
   term_printf("*********************************\r\n");
   term_printf("  Project: %s\r\n", PROJECT_NAME);
   term_printf("  Board  : %s\r\n", TAL_BOARD);
   term_printf("  Version: v%s\r\n", PROJECT_VER_STRING);
   term_printf("  Build  : "__DATE__ " " __TIME__"\r\n");
   term_printf("*********************************\r\n");
   term_printf("\r\n");

} /* OutputBootMessage */

/*************************************************************************/
/*  StartTask                                                            */
/*                                                                       */
/*  This is the Start task.                                              */
/*                                                                       */
/*  In    : task parameter                                               */
/*  Out   : none                                                         */
/*  Return: never                                                        */
/*************************************************************************/
static void StartTask (void *p)
{
   DSTATUS DiskStatus;
   FRESULT Res;

   (void)p;

   /*
    * The StartTask will be used to start all other tasks in the system.
    * At the end the priority will be set to "IDLE" priority.
    */

   OS_SysTickStart();   /* Start the System ticker */
   OS_StatEnable();     /* Enable the statistic function  */

   term_Start();        /* Start the Terminal functionality */
   OutputBootMessage(); /* Output startup messages */

   //OutputFrequencyInfo();
   //OutputUsageInfo();

   /* Initialize SD card */
   disk_initialize(0);

   /*
    * Check if a memory card is available
    */
   DiskStatus = disk_status(0);
   if (DiskStatus & STA_NODISK)
   {
      term_printf("SD Card not found\r\n");
      Reboot();
   }

   /*
    * Mount SD card
    */
   Res = f_mount(&FSObject, LOGICAL_DRIVE, 1);
   if (Res != FR_OK)
   {
      term_printf("SD Card mount error\r\n");
      Reboot();
   }

   /*
    * Copies firmware image from non-volatile memory to RAM
    */
   ImageCopy();

   /*
    * Start firmware
    */
   StartFirmware();

   /*
    * We should never come to this place here
    */
   Reboot();

   OS_TaskChangePriority(TASK_START_PRIORITY_IDLE);

   while (1)
   {
      OS_TimeDly(TASK_START_DELAY_MS);
   }

} /* StartTask */

/*=======================================================================*/
/*  All code exported                                                    */
/*=======================================================================*/

/*************************************************************************/
/*  main                                                                 */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: never                                                        */
/*************************************************************************/
int main (void)
{
   /*
    * Init the Tiny Abstraction Layer
    */
   tal_Init();

   /*
    * Initialize the memory pool
    */
   xmem_Init();

   /*
    * Create the StartTask.
    * The StartTask is the one and only task which
    * will start all other tasks of the system.
    */
   OS_TaskCreate(&TCBStartTask, StartTask, NULL, TASK_START_PRIORITY,
                 StartStack, sizeof(StartStack),
                 "StartTask");

   /*
    * OSStart must be the last function here.
    *
    * Fasten your seatbelt, engine will be started...
    */
   OS_Start();

   /*
    * This return here make no sense.
    * But to prevent the compiler warning:
    *    "return type of 'main' is not 'int'
    * We use an int as return :-)
    */
   return(0); /*lint !e527*/
} /* main */

/*************************************************************************/
/*  term_RxCallback                                                      */
/*                                                                       */
/*  Will be called from TermTask in case a char is received.             */
/*                                                                       */
/*  In    : bData                                                        */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void term_RxCallback (uint8_t bData)
{
   (void)bData;

} /* term_RxCallback */

/*** EOF ***/
