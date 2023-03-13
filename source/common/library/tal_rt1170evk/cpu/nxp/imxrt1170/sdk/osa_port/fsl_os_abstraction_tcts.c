/**************************************************************************
*  Copyright (c) 2021 by Michael Fischer (www.emb4fun.de).
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
*  06.02.2021  mifi  First Version.
**************************************************************************/
#define __FSL_OS_ABSTRACTION_TCTS_C__

/*=======================================================================*/
/*  Include                                                              */
/*=======================================================================*/
#include <stdint.h>
#include <string.h>
#include "tcts.h"

#include "fsl_os_abstraction.h"

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

/*
 * Disable and enable interrupt macros.
 */
 
__attribute__( ( always_inline ) ) static inline uint32_t _DisableAllInts (void)
{
   uint32_t dMask = 0;

   __asm__ volatile ("MRS %0, primask" : "=r" (dMask) );
   __asm__ volatile ("cpsid i" : : : "memory");
  
   return(dMask);
} /* _DisableAllInts */
  
__attribute__( ( always_inline ) ) static inline void _EnableAllInts (uint32_t dMask)
{
   __set_PRIMASK(dMask);
} /* _EnableAllInts */

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

static OS_SEMA  OSASema;
static OS_MUTEX OSAMutex;

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

/*=======================================================================*/
/*  All code exported                                                    */

/*=======================================================================*/

/*************************************************************************/
/*  OSA_EnterCritical                                                    */
/*                                                                       */
/*  Enter critical mode.                                                 */
/*                                                                       */
/*  In    : sr                                                           */
/*  Out   : sr                                                           */
/*  Return: none                                                         */
/*************************************************************************/
void OSA_EnterCritical (uint32_t *sr)
{
   *sr = _DisableAllInts();
} /* OSA_EnterCritical */

/*************************************************************************/
/*  OSA_ExitCritical                                                     */
/*                                                                       */
/*  Exit critical mode and retore the previous mode.                     */
/*                                                                       */
/*  In    : sr                                                           */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void OSA_ExitCritical (uint32_t sr)
{
   _EnableAllInts(sr);
} /* OSA_ExitCritical */

/*************************************************************************/
/*  OSA_TimeDelay                                                        */
/*                                                                       */
/*  This function is used to suspend the active thread for the given     */
/*  number of milliseconds.                                              */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void OSA_TimeDelay (uint32_t millisec)
{
   OS_TimeDly(millisec);

} /* OSA_TimeDelay */

/*************************************************************************/
/*  OSA_SemaphoreCreate                                                  */
/*                                                                       */
/*  Ccreates a semaphore and sets the value to the parameter initValue.  */ 
/*                                                                       */
/*  In    : semaphoreHandle, initValue                                   */
/*  Out   : semaphoreHandle                                              */
/*  Return: KOSA_StatusSuccess / KOSA_StatusError                        */
/*************************************************************************/
osa_status_t OSA_SemaphoreCreate (osa_semaphore_handle_t semaphoreHandle, uint32_t initValue)
{
   static uint8_t InitDone = 0;
   osa_status_t   Status = KOSA_StatusError;

   if ((0 == InitDone) && (semaphoreHandle != NULL))
   {
      InitDone = 1;
      Status   = KOSA_StatusSuccess;

      OS_SemaCreate(&OSASema, (int32_t)initValue, 0xFF);

      *(uint32_t *)semaphoreHandle = (uint32_t)&OSASema;
   }

   return(Status);
} /* OSA_SemaphoreCreate */

/*************************************************************************/
/*  OSA_SemaphoreWait                                                    */
/*                                                                       */
/*  Checks the semaphore's counting value. If it is positive, decreases  */
/*  it and returns KOSA_StatusSuccess. Otherwise, a timeout is used to   */
/*  wait.                                                                */
/*                                                                       */
/*  In    : semaphoreHandle, millisec                                    */
/*  Out   : none                                                         */
/*  Return: KOSA_StatusSuccess / KOSA_StatusError / KOSA_StatusTimeout   */
/*************************************************************************/
osa_status_t OSA_SemaphoreWait (osa_semaphore_handle_t semaphoreHandle, uint32_t millisec)
{
   osa_status_t Status = KOSA_StatusError;
   int          rc;
   uint32_t     TimeoutMs;
   OS_SEMA    *pSema;

   if (semaphoreHandle != NULL)
   {
      TimeoutMs = (osaWaitForever_c == millisec) ? OS_WAIT_INFINITE : millisec;

      pSema = (OS_SEMA*)(*(uint32_t*)semaphoreHandle);
      rc = OS_SemaWait(pSema, TimeoutMs);
      if (OS_RC_OK == rc)
      {
         Status = KOSA_StatusSuccess;  /* Semaphore taken */
      }
      else
      {
         Status = KOSA_StatusTimeout;  /* Timeout */
      }
   }

   return(Status);
} /* OSA_SemaphoreCreate */

/*************************************************************************/
/*  OSA_SemaphorePost                                                    */
/*                                                                       */
/*  Signals for someone waiting on the semaphore to wake up.             */
/*                                                                       */
/*  In    : semaphoreHandle                                              */
/*  Out   : none                                                         */
/*  Return: KOSA_StatusSuccess / KOSA_StatusError                        */
/*************************************************************************/
osa_status_t OSA_SemaphorePost (osa_semaphore_handle_t semaphoreHandle)
{
   osa_status_t Status = KOSA_StatusError;
   OS_SEMA    *pSema;

   if (semaphoreHandle != NULL)
   {
      pSema  = (OS_SEMA*)(*(uint32_t*)semaphoreHandle);
      Status = KOSA_StatusSuccess;

      if (0U != __get_IPSR())
      {
         OS_SemaSignalFromInt(pSema);
      }
      else
      {
         OS_SemaSignal(pSema);
      }
   }

   return(Status);
} /* OSA_SemaphorePost */

/*************************************************************************/
/*  OSA_MutexCreate                                                      */
/*                                                                       */
/*  This function is used to create a mutex.                             */
/*                                                                       */
/*  In    : mutexHandle                                                  */
/*  Out   : none                                                         */
/*  Return: KOSA_StatusSuccess / KOSA_StatusError                        */
/*************************************************************************/
osa_status_t OSA_MutexCreate (osa_mutex_handle_t mutexHandle)
{
   static uint8_t InitDone = 0;
   osa_status_t   Status = KOSA_StatusError;
   
   if (mutexHandle != NULL)
   {
      Status = KOSA_StatusSuccess;

      if (0 == InitDone)
      {
         InitDone = 1;
         Status   = KOSA_StatusSuccess;

         OS_MutexCreate(&OSAMutex);
      }
      
      *(uint32_t *)mutexHandle = (uint32_t)&OSAMutex;
   }      

   return(Status);
} /* OSA_MutexCreate */

/*************************************************************************/
/*  OSA_MutexLock                                                        */
/*                                                                       */
/*  This function checks the mutex's status, if it is unlocked, lock it  */
/*  and returns KOSA_StatusSuccess, otherwise, wait for the mutex.       */
/*  This function returns KOSA_StatusSuccess if the mutex is obtained,   */
/*  returns KOSA_StatusError if any errors occur during waiting. If the  */
/*  mutex has been locked, pass 0 as timeout will return                 */
/*  KOSA_StatusTimeout immediately.                                      */
/*                                                                       */
/*  In    : mutexHandle, millisec                                        */
/*  Out   : none                                                         */
/*  Return: KOSA_StatusSuccess / KOSA_StatusError / KOSA_StatusTimeout   */
/*************************************************************************/
osa_status_t OSA_MutexLock (osa_mutex_handle_t mutexHandle, uint32_t millisec)
{
   osa_status_t Status = KOSA_StatusError;
   int          rc;
   uint32_t     TimeoutMs;
   OS_MUTEX   *pMutex;

   if (mutexHandle != NULL)
   {
      TimeoutMs = (osaWaitForever_c == millisec) ? OS_WAIT_INFINITE : millisec;

      pMutex = (OS_MUTEX*)(*(uint32_t*)mutexHandle);
      rc = OS_MutexWait(pMutex, TimeoutMs);
      if (OS_RC_OK == rc)
      {
         Status = KOSA_StatusSuccess;  /* Semaphore taken */
      }
      else
      {
         Status = KOSA_StatusTimeout;  /* Timeout */
      }
   
   }
   
   return(Status);
} /* OSA_MutexLock */

/*************************************************************************/
/*  OSA_MutexUnlock                                                      */
/*                                                                       */
/*  This function is used to unlock a mutex.                             */
/*                                                                       */
/*  In    : mutexHandle                                                  */
/*  Out   : none                                                         */
/*  Return: KOSA_StatusSuccess / KOSA_StatusError                        */
/*************************************************************************/
osa_status_t OSA_MutexUnlock (osa_mutex_handle_t mutexHandle)
{
   osa_status_t Status = KOSA_StatusError;
   OS_MUTEX   *pMutex;

   if (mutexHandle != NULL)
   {
      pMutex  = (OS_MUTEX*)(*(uint32_t*)mutexHandle);
      Status = KOSA_StatusSuccess;

      if (0U != __get_IPSR())
      {
         while(1)
         {
          /* Not supported */
          __asm__ volatile ("nop");
         }
      }
      else
      {
         OS_MutexSignal(pMutex);
      }
   }

   return(Status);

} /* OSA_MutexUnlock */

/*************************************************************************/
/*  OSA_MutexDestroy                                                     */
/*                                                                       */
/*  This function is used to destroy a mutex.                            */
/*                                                                       */
/*  In    : mutexHandle                                                  */
/*  Out   : none                                                         */
/*  Return: KOSA_StatusSuccess / KOSA_StatusError                        */
/*************************************************************************/
osa_status_t OSA_MutexDestroy (osa_mutex_handle_t mutexHandle)
{
   (void)mutexHandle;
   
   return(KOSA_StatusSuccess);
} /* OSA_MutexDestroy */

/*** EOF ***/
