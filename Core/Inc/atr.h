/**
  ******************************************************************************
  * @file    Smartcard_T1/inc/atr.h 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    13-May-2013
  * @brief   This file contains all the functions prototypes for the ATR library.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _ATR_H
#define _ATR_H

/* Includes ------------------------------------------------------------------*/
#include "legacy.h"
#include "ll_includes.h"
#include "stm32f0xx_hal_conf.h"

/* Exported constants --------------------------------------------------------*/
/* Return values */
#define ATR_OK        0 /* ATR could be parsed and data returned */
#define ATR_NOT_FOUND 1 /* Data not present in ATR */
#define ATR_MALFORMED 2 /* ATR could not be parsed */
#define ATR_IO_ERROR  3 /* I/O stream error */

/* Paramenters */
#define ATR_MAX_SIZE           33 /* Maximum size of ATR byte array */
#define ATR_MAX_HISTORICAL     15 /* Maximum number of historical bytes */
#define ATR_MAX_PROTOCOLS      7 /* Maximun number of protocols */
#define ATR_MAX_IB             4 /* Maximum number of interface bytes per protocol */
#define ATR_CONVENTION_DIRECT  0 /* Direct convention */
#define ATR_CONVENTION_INVERSE 1 /* Inverse convention */
#define ATR_PROTOCOL_TYPE_T0   0 /* Protocol type T=0 */
#define ATR_PROTOCOL_TYPE_T1   1 /* Protocol type T=1 */
#define ATR_PROTOCOL_TYPE_T2   2 /* Protocol type T=2 */
#define ATR_PROTOCOL_TYPE_T3   3 /* Protocol type T=3 */
#define ATR_PROTOCOL_TYPE_T14  14 /* Protocol type T=14 */
#define ATR_INTERFACE_BYTE_TA  0 /* Interface byte TAi */
#define ATR_INTERFACE_BYTE_TB  1 /* Interface byte TBi */
#define ATR_INTERFACE_BYTE_TC  2 /* Interface byte TCi */
#define ATR_INTERFACE_BYTE_TD  3 /* Interface byte TDi */
#define ATR_PARAMETER_F        0 /* Parameter F */
#define ATR_PARAMETER_D        1 /* Parameter D */
#define ATR_PARAMETER_I        2 /* Parameter I */
#define ATR_PARAMETER_P        3 /* Parameter P */
#define ATR_PARAMETER_N        4 /* Parameter N */
#define ATR_INTEGER_VALUE_FI   0 /* Integer value FI */
#define ATR_INTEGER_VALUE_DI   1 /* Integer value DI */
#define ATR_INTEGER_VALUE_II   2 /* Integer value II */
#define ATR_INTEGER_VALUE_PI1  3 /* Integer value PI1 */
#define ATR_INTEGER_VALUE_N    4 /* Integer value N */
#define ATR_INTEGER_VALUE_PI2  5 /* Integer value PI2 */

/* Default parameters values */
#define ATR_DEFAULT_F         372
#define ATR_DEFAULT_D         1
#define ATR_DEFAULT_I         50
#define ATR_DEFAULT_N         0
#define ATR_DEFAULT_P         5

/* Exported types ------------------------------------------------------------*/
typedef struct
{
 uint32_t length;
 uint8_t TS; /* Initial character */
 uint8_t T0; /* Format character */
  
 struct
 {
   uint8_t value;   /* The value of the Interface byte */
   uint8_t present; /* The presence of the Interface byte */
 }
 ib[ATR_MAX_PROTOCOLS][ATR_MAX_IB], TCK; /* Interface bytes, TCK */
  
 uint8_t pn;                            /* Protocol number: the value of i of the Interface bytes */
 uint8_t hb[ATR_MAX_HISTORICAL];        /* Historical bytes table */
 uint8_t hbn;                           /* Historical bytes number */
} ATR_TypeDef;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
uint8_t ATR_Decode(ATR_TypeDef * atr, uint8_t *atr_buffer, uint32_t length);
uint8_t ATR_GetConvention (ATR_TypeDef * atr, uint8_t *convention);
void ATR_GetDefaultProtocol(ATR_TypeDef *atr, int8_t *protocol);
uint8_t ATR_GetIntegerValue (ATR_TypeDef * atr, uint8_t name, uint8_t * value);
uint8_t ATR_GetParameter (ATR_TypeDef * atr, uint8_t name, uint16_t *parameter);

#endif /* _ATR_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
