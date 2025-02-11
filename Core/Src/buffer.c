/**
  ******************************************************************************
  * @file    Smartcard_T1/src/buffer.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    13-May-2013
  * @brief   This file provides all the buffer management for Smartcard applications.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/** @addtogroup SmartCard_T1
  * @{
  */ 


/* Includes ------------------------------------------------------------------*/
#include "buffer.h"
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initialize the buffer.
  * @param  bp: a pointer to the buffer structure.
  * @param  buffer: pointer to the data buffer.  
  * @param  len: the length of the buffer.
  * @retval None.
  */
void Buffer_init(buffer_TypeDef *bp, void *buffer, uint32_t len)
{
  /* void * memset ( void * ptr, int value, size_t num ):
  Sets the first num bytes of the block of memory pointed by ptr to the specified value
  -> Initilaize bp structure to 0 */
  memset(bp, 0, sizeof(*bp)); 
  bp->base = (uint8_t *) buffer;
  bp->size = len;
}

/**
  * @brief  Set a buffer with length = len and initialized to 0.
  * @param  bp: a pointer to the buffer structure.
  * @param  buffer: pointer to the data buffer.  
  * @param  len: the length of the buffer.
  * @retval None
  */
void Buffer_set(buffer_TypeDef *bp, void *buffer, uint32_t len)
{
  Buffer_init(bp, buffer, len);
  bp->tail = len;
}

/**
  * @brief  Get a buffer with length = len.
  * @param  bp: a pointer to the buffer structure.
  * @param  buffer: pointer to the data buffer.  
  * @param  len: the length of the buffer.
  * @retval 
  *     - (>=0) the lenght of the buffer if the operation has succeeded.
  *     - (-1) the operation has not succeeded.
  */
int32_t  Buffer_get(buffer_TypeDef *bp, void *buffer, uint32_t len)
{
  if (len > bp->tail - bp->head)
  {
    return -1;
  }
  if (buffer)
  {
    memcpy(buffer, bp->base + bp->head, len);
  }
  
  bp->head += len;
  return len;
}

/**
  * @brief  put a buffer with length = len.
  * @param  bp: a pointer to the buffer structure.
  * @param  buffer: pointer to the data buffer.  
  * @param  len: the length of the buffer.
  * @retval 
  *     - (>=0) the lenght of the buffer if the operation has succeeded.
  *     - (-1) the operation has not succeeded.
  */
int32_t  Buffer_put(buffer_TypeDef *bp, const void *buffer, uint32_t len)
{
  if (len > bp->size - bp->tail)
  {
    bp->overrun = 1;
    return -1;
  }
  if (buffer)
  {
    memcpy(bp->base + bp->tail, buffer, len);
  }
  
  bp->tail += len;
  return len;
}

/**
  * @brief  put a data byte in a buffer.
  * @param  bp: a pointer to the buffer structure.
  * @param  byte: the data byte.
  * @retval 
  *     -  1  if the operation has succeeded.
  *     - (-1) the operation has not succeeded.
  */
int32_t  Buffer_putc(buffer_TypeDef *bp, uint8_t byte)
{
  uint8_t c = byte;
  
  return Buffer_put(bp, &c, 1);
}

/**
  * @brief  Returns the lenght of the available buffer.
  * @param  bp: a pointer to the buffer structure.
  * @retval the lenght of the available buffer.
  */
uint32_t Buffer_avail(buffer_TypeDef *bp)
{
  return (bp->tail - bp->head);
}

/**
  * @brief  Returns the head address of the buffer.
  * @param  bp: a pointer to the buffer structure.
  * @retval the head address of the buffer.
  */
void * Buffer_head(buffer_TypeDef *bp)
{
  return (bp->base + bp->head);
}

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
