/**
  ******************************************************************************
  * @file    SmartCard_T0/inc/platform_config.h 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    13-May-2013
  * @brief   Evaluation board specific configuration file.
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
#ifndef __PLATFORM_CONFIG_H
#define __PLATFORM_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Exported types ------------------------------------------------------------*/
typedef enum
{
  Bit_RESET = GPIO_PIN_RESET,
  Bit_SET = GPIO_PIN_SET
}BitAction;
/* Exported constants --------------------------------------------------------*/

/* Define the STM32F0xx hardware used from STM320518_EVAL evaluation board */
#define SC_USART                           USART1
#define SC_USART_CLK                       LL_APB1_GRP2_PERIPH_USART1
#define SC_USART_APBPERIPHCLOCK            RCC_APB2PeriphClockCmd
#define SC_USART_IRQn                      USART1_IRQn
#define SC_USART_IRQHandler                USART1_IRQHandler

#define SC_USART_TX_PIN                    LL_GPIO_PIN_9                
#define SC_USART_TX_GPIO_PORT              GPIOA                       
#define SC_USART_TX_GPIO_CLK               LL_AHB1_GRP1_PERIPH_GPIOA
#define SC_USART_TX_SOURCE                 LL_GPIO_PIN_9
#define SC_USART_TX_AF                     LL_GPIO_AF_1

#define SC_USART_CK_PIN                    LL_GPIO_PIN_8                
#define SC_USART_CK_GPIO_PORT              GPIOA                    
#define SC_USART_CK_GPIO_CLK               LL_AHB1_GRP1_PERIPH_GPIOA
#define SC_USART_CK_SOURCE                 LL_GPIO_PIN_8
#define SC_USART_CK_AF                     LL_GPIO_AF_1


/* Smartcard Inteface GPIO pins */
#define SC_3_5V_PIN                        LL_GPIO_PIN_12
#define SC_3_5V_GPIO_PORT                  GPIOA
#define SC_3_5V_GPIO_CLK                   LL_AHB1_GRP1_PERIPH_GPIOA

#define SC_RESET_PIN                       LL_GPIO_PIN_15
#define SC_RESET_GPIO_PORT                 GPIOB
#define SC_RESET_GPIO_CLK                  LL_AHB1_GRP1_PERIPH_GPIOB

#define SC_CMDVCC_PIN                      LL_GPIO_PIN_13
#define SC_CMDVCC_GPIO_PORT                GPIOC
#define SC_CMDVCC_GPIO_CLK                 LL_AHB1_GRP1_PERIPH_GPIOC

#define SC_OFF_PIN                         LL_GPIO_PIN_11
#define SC_OFF_GPIO_PORT                   GPIOA
#define SC_OFF_GPIO_CLK                    LL_AHB1_GRP1_PERIPH_GPIOA
#define SC_OFF_EXTI_LINE                   LL_EXTI_LINE_11
#define SC_OFF_EXTI_PORT_SOURCE            LL_SYSCFG_EXTI_PORTA
#define SC_OFF_EXTI_PIN_SOURCE             LL_SYSCFG_EXTI_LINE11
#define SC_OFF_EXTI_IRQn                   EXTI4_15_IRQn 
#define SC_OFF_EXTI_IRQHandler             EXTI4_15_IRQHandler

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __PLATFORM_CONFIG_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
