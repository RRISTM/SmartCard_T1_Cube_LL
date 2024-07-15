/**
  ******************************************************************************
  * @file    Smartcard_T1/src/t1_protocol_param.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    13-May-2013
  * @brief   This file provides some functions for T=1 protocol parameters
  *          configuration.
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
#include "t1_protocol_param.h"
#include "t1_hal.h"
#include "pps.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Set the smartcard baudrate function of F and D parameters and apply
  *         PPS procedure.
  * @param  SC_atr: a pointer to the ATR structure.
  * @param  SC_clk: the smartcard clock frequency in Hz.  
  * @retval The etu value in us.
  */
uint32_t Set_F_D_parameters(ATR_TypeDef* SC_atr, uint32_t SC_clk)   
{
   uint8_t PPS_Response_length = 0, pps1 = 0; 
   uint8_t PPS_buffer[4] = {0xFF, 0x11, 0x45, 0xAB};
   uint32_t baud;
   uint16_t F=372;
   uint16_t D=1;
   uint8_t TA2=0;
   /* ((Fd/Dd)/SC_clk) * 1000000 (in us) (+1 to get the ceil of the value)*/
   uint32_t etu = (372000000/SC_clk)+1; 
   
   /* TA2 is present ? */
    if (SC_atr->ib[1][ATR_INTERFACE_BYTE_TA].present)
    {
       
       /* ---TA2 is present with value: TA2 (the SmartCard is in specific mode)--- */
       TA2 = SC_atr->ib[1][ATR_INTERFACE_BYTE_TA].value;
      
       /* TA2 b5=0: Fi and Di should be applyed after ATR */
       if((TA2 & 0x10) == 0) 
       {
          (void)ATR_GetParameter(SC_atr, ATR_PARAMETER_D, &D);
          (void)ATR_GetParameter(SC_atr, ATR_PARAMETER_F, &F);
          
           Compute_etu_baudrate(F, D, SC_clk, &etu, &baud);
        
           /* Configure the usart with the new baudrate */
           SC_USART_Baud_Config(baud);
        }
     }
     else
     {
       /* ---TA2 is absent: the SmartCard is in negotiable mode--- */
     
        /* TA1 is present ? (contains FI and DI)*/
        if (SC_atr->ib[0][ATR_INTERFACE_BYTE_TA].present)
        {
           /* If the card supports values different from default values */
           if(SC_atr->ib[0][ATR_INTERFACE_BYTE_TA].value != 0x11)
           {
              /* PPS1 */
              PPS_buffer[PPS1] = SC_atr->ib[0][ATR_INTERFACE_BYTE_TA].value;
         
              /* Apply PPS (Protocol Paramaters Selection) */
              (void)PPS_Exchange (PPS_buffer, &PPS_Response_length, &pps1, etu);
         
              D = pps1 & 0xF;
              F = pps1 >> 4;
              
              Compute_etu_baudrate(F, D, SC_clk, &etu, &baud);
          
              /* Configure the usart with the new baudrate */
              SC_USART_Baud_Config(baud);
        
               /* ---TA1 is present with value: SC_atr->ib[0][ATR_INTERFACE_BYTE_TA].value--- */
               /* ---New SmartCard baudrate: baud--- */
           }
           else
           {
               /* ---TA1 is absent (use of default values F=372 and D=1)--- */
               /* ---SmartCard still having the same baudrate: baud--- */
           }
        }
      }
     
    return etu;
}


/**
  * @brief  Get the IFSC paramater from ATR and set it to t1 structure.
  * @param  SC_atr: a pointer to the ATR structure.
  * @param  t1: pointer to t1 stucture of T=1 protocol.
  * @retval None.
  */
void Set_IFSC(ATR_TypeDef * SC_atr, t1_TypeDef* t1)
{
   __IO uint32_t i;
   uint8_t ifsc = 5;
  
   /* TAi (i>2) present? (see 9.5.2.1: IFS for the card) */
   for (i=2; i<ATR_MAX_PROTOCOLS; i++)
   {
       if (SC_atr->ib[i][ATR_INTERFACE_BYTE_TA].present)
       {
	  ifsc = SC_atr->ib[i][ATR_INTERFACE_BYTE_TA].value;
                              
          /* only the first TAi (i>2) must be used */
	  break;
	}
   }
      
   (void)T1_SetParameter(t1, T1_PROTOCOL_IFSC, ifsc);
}

/**
  * @brief  Get the CWT and BWT paramaters from ATR and set them to t1 structure.
  * @param  SC_atr: a pointer to the ATR structure.
  * @param  t1: pointer to t1 stucture of T=1 protocol.
  * @retval None.
  */
void Set_CWT_BWT(ATR_TypeDef* SC_atr, t1_TypeDef* t1)
{
   __IO uint32_t i; 
   uint8_t waiting_times = 0;
   uint8_t cwi = 13;
   uint8_t bwi = 4;
   
     /* TBi (i>2) present? (see 9.5.3: Waiting times (CWT and BWT)) */
      for (i=2; i<ATR_MAX_PROTOCOLS; i++)
      {
	  if (SC_atr->ib[i][ATR_INTERFACE_BYTE_TB].present)
	  {
	     waiting_times = SC_atr->ib[i][ATR_INTERFACE_BYTE_TB].value;
             cwi = waiting_times&0xF;
             bwi = waiting_times>>4;
                            
	     /* only the first TAi (i>2) must be used */
	     break;
	  }
      }
     /* Set BWI value */
     (void)T1_SetParameter(t1, T1_PROTOCOL_BWI, bwi);
     /* Set CWI value */
     (void)T1_SetParameter(t1, T1_PROTOCOL_CWI, cwi);
}

/**
  * @brief  Get the extra-guardtime paramater from ATR and configure the USART
  *     guardtime.
  * @param  SC_atr: a pointer to the ATR structure.
  * @retval None.
  */
void Set_EGT(ATR_TypeDef* SC_atr)
{
       uint16_t N;
  
      /* TC1 is present ? (contains the extra guardtime) */
      if (SC_atr->ib[0][ATR_INTERFACE_BYTE_TC].present)
      {
         (void)ATR_GetParameter (SC_atr, ATR_PARAMETER_N, &N);
     
         if(N>1) /* There is 1.5 extra guard time added by the USART */
         {
            /* USART Guard Time set to 16 Bit */
            LL_USART_SetSmartcardGuardTime(USART1, (uint8_t)N-1);
         }
         else
         {
            /* USART Guard Time set to 16 Bit */
            LL_USART_SetSmartcardGuardTime(USART1, 0);
         }
         /* ---TC1 is present with value: N (extra guardtime is used)--- */     
      }
      else
      {
         /* ---TC1 is absent (no extra guardtime is used)--- */
      }
}

/**
  * @brief  Get the type of checksum to be used from ATR and set it to t1 structure.
  * @param  SC_atr: a pointer to the ATR structure.
  * @param  t1: pointer to t1 stucture of T=1 protocol.
  * @retval None.
  */
void Set_EDC(ATR_TypeDef * SC_atr, t1_TypeDef* t1)
{
  __IO uint32_t i;
  
  uint8_t edc_type = T1_PROTOCOL_CHECKSUM_LRC;
  
  /* TCi (i>2) present? (see 9.5.4: Error detection code) */
  for (i=2; i<ATR_MAX_PROTOCOLS; i++)
  {
    if (SC_atr->ib[i][ATR_INTERFACE_BYTE_TC].present)
    {                
      /* b1 = 0 -> LRC*/
      if (SC_atr->ib[i][ATR_INTERFACE_BYTE_TC].value == 0)
      {
        edc_type = T1_PROTOCOL_CHECKSUM_LRC;
      }
      /* b1 = 1 -> CRC*/
      else if (SC_atr->ib[i][ATR_INTERFACE_BYTE_TC].value == 1)
      {
        edc_type = T1_PROTOCOL_CHECKSUM_CRC;
      }
      else
      {
        /* ---Wrong value for TCi--- */
        break;
      }  
      
      /* only the first TCi (i>2) must be used */
      break;
    }
  } 
  
  (void)T1_SetParameter(t1, edc_type, 0);
}

/**
  * @brief  Get the convention to be used from ATR and set it to t1 structure.
  * @param  SC_atr: a pointer to the ATR structure.
  * @param  t1: pointer to t1 stucture of T=1 protocol.
  * @retval None.
  */
void Set_Convention(ATR_TypeDef * SC_atr, t1_TypeDef* t1)
{
  uint8_t convention = ATR_CONVENTION_DIRECT;
  
  (void)ATR_GetConvention (SC_atr,&convention);
  
  if(convention == ATR_CONVENTION_INVERSE)
  {
    (void)T1_SetParameter(t1, T1_PROTOCOL_CONVENTION, INVERSE_CONV);
    
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
