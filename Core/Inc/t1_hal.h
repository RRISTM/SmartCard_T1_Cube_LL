/**
  ******************************************************************************
  * @file    Smartcard_T1/inc/t1_hal.h 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    13-May-2013
  * @brief   This file contains all the functions prototypes for the T=1 hal
  *          library.
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
#ifndef __T1_HAL_H
#define __T1_HAL_H

/* Includes ------------------------------------------------------------------*/
#include "legacy.h"
#include "ll_includes.h"
#include "stm32f0xx_hal_conf.h"
#include "t1_protocol.h"
#include "platform_config.h"

/* Exported constants --------------------------------------------------------*/
#define SC_CLK_MAX           4000000 /* Clock max in Hz supported by the card */

/* Smartcard Voltage */
#define SC_Voltage_5V        0
#define SC_Voltage_3V        1

#define T1_RECEIVE_SUCCESS   0
#define T1_BWT_TIMEOUT      -1
#define T1_PPS_TIMEOUT      -1
#define T1_PARITY_ERROR     -2

#define DIRECT_CONV          0
#define INVERSE_CONV         1

/* Exported types ------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void SC_HardwareInit(void);
void SC_Reset(BitAction ResetState);
void SC_PowerCmd(FunctionalState NewState);
void SC_VoltageConfig(uint32_t SC_Voltage);
uint32_t SC_USARTConfig(uint32_t *etu_us, uint32_t *baud);
void SC_USART_Baud_Config(uint32_t baud);
void Compute_etu_baudrate(uint32_t F, uint32_t D, uint32_t frequency, uint32_t *etu, uint32_t *baudrate);
/* For ATR */
void SC_GetATR(uint8_t *ATR_buffer, uint32_t etu_us, uint32_t max_length);
/* For PPS exchange */
void PPS_Transmit(uint8_t * request, uint8_t len_request, uint32_t etu_usec);
int32_t PPS_Receive(uint8_t * response, uint8_t *len_response);
/* For T=1 protocol */
void T1_USART_TxRx_Handler(void);
void T1_TimingDecrement(void);
int32_t T1_TxRxBlock(t1_TypeDef *t1, uint8_t *txrxBuffer, uint32_t tx_length);

#endif /* __T1_HAL_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
