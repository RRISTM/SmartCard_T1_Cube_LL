/**
  ******************************************************************************
  * @file    Smartcard_T1/src/atr.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    13-May-2013
  * @brief   This file provides all ATR functions for Smartcard applications.
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
#include "atr.h"
#include <string.h>


/* Private typedef -----------------------------------------------------------*/
#define PROTOCOL_UNSET -1

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Interface bytes table */
static const uint16_t atr_num_ib_table[16] = {
  0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4
};

/* Fi table */
static const uint16_t atr_f_table[16] = {
  372, 372, 558, 744, 1116, 1488, 1860, 0, 0, 512, 768, 1024, 1536, 2048, 0, 0
};

/* Di table */
static const uint16_t atr_d_table[16] = {
  0, 1, 2, 4, 8, 16, 32, 64, 12, 20, 0, 0, 0, 0, 0, 0
};

/* I table (progamming current) */
static const uint16_t atr_i_table[4] = {
  25, 50, 100, 0
};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Decode the received ATR response buffer.
  * @param  atr: pointer to ATR stucture.
  * @param  atr_buffer: a pointer to a buffer that contains ATR frame.
  * @param  length: the length of the ATR to be decoded.
  * @retval the answer to reset status. The returned value can
  *   be one of the following: 
  *          - ATR_OK: Answer To Reset is well formed.
  *          - ATR_MALFORMED: Answer To Reset is malformed.
  */
uint8_t ATR_Decode(ATR_TypeDef * atr, uint8_t *atr_buffer, uint32_t length)
{
  uint8_t TDi;
  uint8_t j=0;
  uint8_t tck_atr=0;
  uint32_t pointer = 0, pn = 0;

  /* Check size of buffer */
  if (length < 2)
  {
      return (ATR_MALFORMED);
  }

  /* Check size of buffer */
  if ((atr_buffer[0] != 0x3B) && (atr_buffer[0] != 0x3F))
  {
      return (ATR_MALFORMED);
  } 
  
  /* Store TS */
  atr->TS = atr_buffer[0];

  /* Store T0 */ 
  TDi = atr_buffer[1];
  atr->T0 = TDi;
     
  /* Store number of historical bytes */
  atr->hbn = TDi & 0x0F;

  /* TCK is not present by default */
  (atr->TCK).present = 0;
  
  pointer = 1;

  /* Extract interface bytes */
  while (pointer < length)
  {
      /* Check buffer is long enought */
      if (pointer + atr_num_ib_table[((uint32_t)0xF0 & TDi) >> 4] >= length)
      {
	  return (ATR_MALFORMED);
      }
      
      /* Check TAi is present */
      if ((TDi | 0xEF) == 0xFF)
      {
	 pointer++;
	 atr->ib[pn][ATR_INTERFACE_BYTE_TA].value = atr_buffer[pointer];
	 atr->ib[pn][ATR_INTERFACE_BYTE_TA].present = 1;
      }
      else
      {
	 atr->ib[pn][ATR_INTERFACE_BYTE_TA].present = 0;
      }
      
      /* Check TBi is present */
      if ((TDi | 0xDF) == 0xFF)
      {
	 pointer++;
	 atr->ib[pn][ATR_INTERFACE_BYTE_TB].value = atr_buffer[pointer];
	 atr->ib[pn][ATR_INTERFACE_BYTE_TB].present = 1;
      }
      else
      {
	 atr->ib[pn][ATR_INTERFACE_BYTE_TB].present = 0;
      }

      /* Check TCi is present */
      if ((TDi | 0xBF) == 0xFF)
      {
	 pointer++;
	 atr->ib[pn][ATR_INTERFACE_BYTE_TC].value = atr_buffer[pointer];
	 atr->ib[pn][ATR_INTERFACE_BYTE_TC].present = 1;
      }
      else
      {
	 atr->ib[pn][ATR_INTERFACE_BYTE_TC].present = 0;
      }

      /* Read TDi if present */
      if ((TDi | 0x7F) == 0xFF)
      {
	 pointer++;
	 TDi = atr->ib[pn][ATR_INTERFACE_BYTE_TD].value = atr_buffer[pointer];
	 atr->ib[pn][ATR_INTERFACE_BYTE_TD].present = 1;
	 (atr->TCK).present = ((TDi & 0x0F) != ATR_PROTOCOL_TYPE_T0);
	  
	 if (pn >= ATR_MAX_PROTOCOLS)
	 {
	   return (ATR_MALFORMED);
	 }
	  
	 pn++;
      }
      else
      {
         atr->ib[pn][ATR_INTERFACE_BYTE_TD].present = 0;
	 break;
      }
  }

  /* Store number of protocols */
  atr->pn = pn + 1;

  /* Store historical bytes */
  if (pointer + atr->hbn >= length)
  {
    return (ATR_MALFORMED);
  }

  memcpy (atr->hb, atr_buffer + pointer + 1, atr->hbn);
  pointer += (atr->hbn);

  /* Store TCK  */
  if ((atr->TCK).present)
  {
     if (pointer + 1 >= length)
     {
	return (ATR_MALFORMED);
     }
     
     pointer++;

     (atr->TCK).value = atr_buffer[pointer];
  }

  atr->length = pointer + 1;

  if ((atr->TCK).present)
  {  
     tck_atr = atr_buffer[1];
   
     for(j=2;j< (atr->length - 2);j++)
     {
        tck_atr ^= atr_buffer[j];
     }
     if(tck_atr != atr_buffer[atr->length-1])
     {
        return (ATR_MALFORMED);
     }
  }
 
  return (ATR_OK);
}

/**
  * @brief  Get the convention type from the ATR structure.
  * @param  atr: a pointer to ATR stucture.
  * @param  convention: a pointer to a variable that contains the convention type.
  * @retval the answer to reset status. The returned value can
  *   be one of the following: 
  *          - ATR_OK: Answer To Reset is well formed (convention is OK).
  *          - ATR_MALFORMED: Answer To Reset is malformed (convention is not OK).
  */
uint8_t ATR_GetConvention (ATR_TypeDef * atr, uint8_t *convention)
{
  if (atr->TS == 0x3B)
  {
     (*convention) = ATR_CONVENTION_DIRECT;
  }
  else if (atr->TS == 0x3F)
  {
     (*convention) = ATR_CONVENTION_INVERSE;
  }
  else
  {
     return (ATR_MALFORMED);
  }
  
  return (ATR_OK);
}

/**
  * @brief  Get the integer value of the selected parameter.
  * @param  atr: a pointer to ATR stucture.
  * @param  name: the name of the parameter.
  *   This parameter can be one of the following values:
  *     @arg ATR_INTEGER_VALUE_FI: FI value.
  *     @arg ATR_INTEGER_VALUE_DI: DI value.
  *     @arg ATR_INTEGER_VALUE_II: maximum programming current.
  *     @arg ATR_INTEGER_VALUE_PI1: programming voltage 1.
  *     @arg ATR_INTEGER_VALUE_PI2: programming voltage 2.
  *     @arg ATR_INTEGER_VALUE_N: extra guard time.
  * @param  value: a pointer to the parameter value.
  * @retval the availiability of the paramater in the ATR. The returned value 
  *   can be one of the following:
  *          - ATR_OK: the parameter was found in the ATR.
  *          - ATR_NOT_FOUND: the pareameter was not found in the ATR.
  */
uint8_t ATR_GetIntegerValue (ATR_TypeDef * atr, uint8_t name, uint8_t * value)
{
  uint8_t ret;

  if (name == ATR_INTEGER_VALUE_FI)  /* TA1: b8 to b5 */
  {
      if (atr->ib[0][ATR_INTERFACE_BYTE_TA].present)
      {
	  (*value) = (atr->ib[0][ATR_INTERFACE_BYTE_TA].value & (uint32_t)0xF0) >> 4;
	  ret = ATR_OK;
      }
      else
      {
	  ret = ATR_NOT_FOUND;
      }
  }
  else if (name == ATR_INTEGER_VALUE_DI) /* TA1: b4 to b1 */
  {
      if (atr->ib[0][ATR_INTERFACE_BYTE_TA].present)
      {
	  (*value) = (atr->ib[0][ATR_INTERFACE_BYTE_TA].value & 0x0F);
	  ret = ATR_OK;
      }
      else
      {
	  ret = ATR_NOT_FOUND;
      }
  }
  else if (name == ATR_INTEGER_VALUE_II) /* TB1: b7 to b6 */
  {
      if (atr->ib[0][ATR_INTERFACE_BYTE_TB].present)
      {
	  (*value) = (atr->ib[0][ATR_INTERFACE_BYTE_TB].value & (uint32_t)0x60) >> 5;
	  ret = ATR_OK;
      }
      else
      {
	  ret = ATR_NOT_FOUND;
      }
  }
  else if (name == ATR_INTEGER_VALUE_PI1) /* TB1: b5 to b1 */
  {
      if (atr->ib[0][ATR_INTERFACE_BYTE_TB].present)
      {
	  (*value) = (atr->ib[0][ATR_INTERFACE_BYTE_TB].value & 0x1F);
	  ret = ATR_OK;
      }
      else
      {
	  ret = ATR_NOT_FOUND;
      }
  }
  else if (name == ATR_INTEGER_VALUE_PI2) /* TB2: b8 to b1 */
  {
      if (atr->ib[1][ATR_INTERFACE_BYTE_TB].present)
      {
	  (*value) = atr->ib[1][ATR_INTERFACE_BYTE_TB].value;
	  ret = ATR_OK;
      }
      else
      {
	  ret = ATR_NOT_FOUND;
      }
  }
  else if (name == ATR_INTEGER_VALUE_N) /* TC1: b8 to b1 */
  {
      if (atr->ib[0][ATR_INTERFACE_BYTE_TC].present)
      {
	  (*value) = atr->ib[0][ATR_INTERFACE_BYTE_TC].value;
	  ret = ATR_OK;
      }
      else
      {
	  ret = ATR_NOT_FOUND;
      }
  }
  else
  {
     ret = ATR_NOT_FOUND;
  }

   return ret;
}

/**
  * @brief  Get the selected parameter.
  * @param  atr: a pointer to ATR stucture.
  * @param  name: the name of the parameter.
  *   This parameter can be one of the following values:
  *     @arg ATR_PARAMETER_F: F value.
  *     @arg ATR_PARAMETER_D: D value.
  *     @arg ATR_PARAMETER_I: maximum programming current.
  *     @arg ATR_PARAMETER_P: maximum programming voltage.
  *     @arg ATR_PARAMETER_N: guard time value.
  * @param  parameter: a pointer to the parameter value.
  * @retval the availiability of the paramater in the ATR. The returned value 
  *   can be one of the following:
  *          - ATR_OK: the parameter was found in the ATR.
  *          - ATR_NOT_FOUND: the parameter was not found in the ATR.
  */
uint8_t ATR_GetParameter (ATR_TypeDef * atr, uint8_t name, uint16_t *parameter)
{
  uint8_t FI, DI, II, PI1, PI2, N;

  if (name == ATR_PARAMETER_F)          /* Get F parameter */
  {
      if (ATR_GetIntegerValue (atr, ATR_INTEGER_VALUE_FI, &FI) == ATR_OK)
      {
	(*parameter) = atr_f_table[FI];
      }
      else
      {
	(*parameter) = (uint16_t)ATR_DEFAULT_F;
      }
      return (ATR_OK);
  }
  else if (name == ATR_PARAMETER_D)    /* Get D parameter */
  {
      if (ATR_GetIntegerValue (atr, ATR_INTEGER_VALUE_DI, &DI) == ATR_OK)
      {  
	(*parameter) = atr_d_table[DI];
      }
      else
      {
	(*parameter) = (uint16_t)ATR_DEFAULT_D;
      }
      return (ATR_OK);
  }
  else if (name == ATR_PARAMETER_I)   /* Get I parameter */
  {
      if (ATR_GetIntegerValue (atr, ATR_INTEGER_VALUE_II, &II) == ATR_OK)
      {
	(*parameter) = (atr_i_table[II]);
      }
      else
      {
	(*parameter) = (uint16_t)ATR_DEFAULT_I;
      }
      return (ATR_OK);
  }
  else if (name == ATR_PARAMETER_P)  /* Get P parameter */
  {
      if (ATR_GetIntegerValue (atr, ATR_INTEGER_VALUE_PI2, &PI2) == ATR_OK)
      {
	(*parameter) = (uint16_t) PI2;
      }
      else if (ATR_GetIntegerValue (atr, ATR_INTEGER_VALUE_PI1, &PI1) == ATR_OK)
      {
	(*parameter) = (uint16_t) PI1;
      }
      else
      {
	(*parameter) = (uint16_t) ATR_DEFAULT_P;
      }
      return (ATR_OK);
  }
  else if (name == ATR_PARAMETER_N)   /* Get guard time parameter */
  {
      if (ATR_GetIntegerValue (atr, ATR_INTEGER_VALUE_N, &N) == ATR_OK)
      {
	(*parameter) = (uint16_t) N;
      }
      else
      {
	(*parameter) = (uint16_t) ATR_DEFAULT_N;
      }
      return (ATR_OK);
  }

  return (ATR_NOT_FOUND);
}

/**
  * @brief  Get default protocol.
  * @param  atr: a pointer to ATR stucture.
  * @param  protocol: a pointer to the default protocol value.
  * @retval None.
  */
void ATR_GetDefaultProtocol(ATR_TypeDef *atr, int8_t *protocol)
{
	int32_t i;

	/* default value */
	*protocol = PROTOCOL_UNSET;

	for (i=0; i<ATR_MAX_PROTOCOLS; i++)
        {
	    if ((atr->ib[i][ATR_INTERFACE_BYTE_TD].present) && (PROTOCOL_UNSET == *protocol))
	    {
	        /* set to the first protocol byte found */
	       *protocol = atr->ib[i][ATR_INTERFACE_BYTE_TD].value & 0x0F;
#ifdef DEBUG_                
		DEBUG("Default protocol: T=%d\n\r", *protocol);
#endif                
	    }
        }

	/* specific mode if TA2 present */
	if (atr->ib[1][ATR_INTERFACE_BYTE_TA].present)
	{
	    *protocol = atr->ib[1][ATR_INTERFACE_BYTE_TA].value & 0x0F;
#ifdef DEBUG_            
	     DEBUG("Specific mode found: T=%d\n\r", *protocol);
#endif             
	}

	if (PROTOCOL_UNSET == *protocol)
	{
#ifdef DEBUG	     
            DEBUG("No default protocol found in ATR. Using T=0\n\r");
#endif           
	    *protocol = ATR_PROTOCOL_TYPE_T0;
	}
}

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
