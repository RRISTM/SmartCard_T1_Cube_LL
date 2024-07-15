/**
  ******************************************************************************
  * @file    Smartcard_T1/src/pps.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    13-May-2013
  * @brief   This file provides all the PPS functions for Smartcard applications.
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
#include "pps.h"
#include "t1_hal.h"
#include <string.h>


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static uint8_t PPS_Match (uint8_t* request, uint8_t len_request, uint8_t* response, uint8_t len_response);
static uint8_t PPS_GetLength (uint8_t* pps_buffer);
static uint8_t PPS_GetPCK (uint8_t* pps_buffer, uint8_t length);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  apply the PPS procedure.
  * @param  request: a pointer to the PPS request buffer.
  * @param  resp_length: a pointer to a variable that will contain the PPS 
  *         response length.
  * @param  pps1: pointer to a PPS1 value.
  * @param  etu: elementary time unit in us.
  * @retval The status of the PPS exchange. The returned value can
  *   be one of the following:
  *     - PPS_OK: PPS response matched with the PPS request.
  *     - PPS_HANDSAKE_ERROR: PPS response not matched with the PPS request.
  *     - PPS_COMMUNICATION_ERROR: communication failed.
  */
int32_t PPS_Exchange(uint8_t* request, uint8_t* resp_length, uint8_t* pps1, uint32_t etu)
{
  uint8_t response[PPS_MAX_LENGTH];
  uint8_t len_request, len_response;
  int32_t ret;
  
  request[0] = 0xFF;
  
  len_request = PPS_GetLength (request);
  
  /* Compute the check Character (PCK) */
  request[len_request - 1] = PPS_GetPCK(request, len_request - 1);
  
  /* ---PPS: Sending request: request, len_request--- */
  PPS_Transmit(request, len_request, etu);
  
  if(PPS_Receive(response, &len_response) != 0)
  {
    /* Communication error: Response timout or parity error */
    return PPS_COMMUNICATION_ERROR; 
  }
  
  /* ---PPS: Receiving response: ", response, len_response--- */
  if (!PPS_Match (request, len_request, response, len_response))
  {
    /* The response does not match with the request */
    ret = PPS_HANDSAKE_ERROR; 
  }
  else
  {   
    ret = PPS_OK;  /* PPS response matched with the PPS request */
  }
  
  /* If PPS1 is echoed */
  if (PPS_HAS_PPS1 (request) && PPS_HAS_PPS1 (response))
  {     
    *pps1 = response[PPS1];
  }
  else
  {
    /* ---PPS1 is not present, the default value used: 11--- */
    /* default TA1 */   
    *pps1 = 0x11;	
  }
  
  /* Copy PPS handsake in request buffer */
  memcpy (request, response, len_response);
  
  (*resp_length) = len_response;
  
  return ret;
}

/**
  * @brief  Compare the PPS request to the PPS response.
  * @param  request: a pointer to the PPS request buffer.
  * @param  len_request: the length of the PPS request buffer.
  * @param  response: pointer to a PPS response buffer.
  * @param  len_response: the length of the PPS request buffer.
  * @retval The status of the PPS comparaison. The returned value can
  *   be one of the following:
  *     - 1: PPS response matched with PPS request.
  *     - 0: PPS response did not match with the PPS request.
  */
static uint8_t PPS_Match (uint8_t* request, uint8_t len_request, uint8_t* response, uint8_t len_response)
{
  static __IO uint8_t i = 0;
  uint8_t pck = 0;
  uint8_t len_resp=0;
  
    
  /* Response longer than request */
  if (len_request < len_response)
  {	
    return 0;
  }
   
  /* The bits b1 to b4 of PPS0 response should echoes b1 to b4 PPS0 request */
  if (response[PPSS] !=  0xFF)	
  {
     return 0;
  }
  
  /* The bits b1 to b4 of PPS0 response should echoes b1 to b4 PPS0 request */
  if ((request[PPS0] & 0xF) != (response[PPS0] & 0xF))	
  {
     return 0;
  }
  
  /* See if the reply differs from request: */
  /* Same length  and different contents */ 
  if ((len_request == len_response) && (memcmp(request, response, len_request)))	
  {
     return 0;
  }
  
  /* See if the card specifies other than default FI and DI */
  if ((PPS_HAS_PPS1 (response)) && (response[PPS1] != request[PPS1]))
  {
    return 0;
  }
  
  len_resp = PPS_GetLength (response);
  
  pck = response[0];
  /* Compute PCK */
  for(i=1; i<(len_resp-1); i++)
  {
     pck ^= response[i];
  }

  if(pck != response[len_resp-1])
  {
    return 0;
  }
  
  /* The PPS response mathes with PPS request */
  return 1;
}

/**
  * @brief  compute the length of the PPS frame.
  * @param  pps_buffer: pointer to the PPS buffer.
  * @retval the length of the PPS frame.
  */
static uint8_t PPS_GetLength (uint8_t* pps_buffer)
{
  uint8_t length = 3; /* Minimum 3bytes: PPSS, PPS0 and PCK */

  if (PPS_HAS_PPS1 (pps_buffer))
  {
     length++;
  }

  if (PPS_HAS_PPS2 (pps_buffer))
  {
     length++;
  }

  if (PPS_HAS_PPS3 (pps_buffer))
  {
     length++;
  }

  return length;
}

/**
  * @brief  Compute PCK of the PPS frame.
  * @param  pps_buffer: pointer to the PPS buffer.
  * @param  length: the length of the PPS buffer.  
  * @retval the PCK value.
  */
static uint8_t PPS_GetPCK (uint8_t* pps_buffer, uint8_t length)
{
  uint8_t pck;
  uint8_t i;

  pck = pps_buffer[0];
  
  for (i = 1; i < length; i++)
  {
    pck ^= pps_buffer[i];
  }

  return pck;
}

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
