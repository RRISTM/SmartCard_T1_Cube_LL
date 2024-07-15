/**
  ******************************************************************************
  * @file    Smartcard_T1/src/t1_hal.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    13-May-2013
  * @brief   This file provides all the hal functions for T=1 Smartcard type on
  *          STM32 microcontrollers platform.
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
#include "t1_hal.h"
#include "t1_protocol.h"
#include <string.h>


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SC_VOLTAGE_5V      0
#define SC_VOLTAGE_3V      1
#define K_BWT              960 * 372 * 1000000 /* BWT Constant */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint32_t DataRxIndex = 0;
__IO uint32_t TxBufferIndex = 0;
__IO uint32_t ParityErrorCount = 0;
__IO uint32_t BlockReceiveDoneFlag = 0;
__IO uint32_t pps_exchange_flag = 0;
__IO uint32_t Bwt_us, Cwt_us = 0xFA0;  /*default value 4s*/
uint32_t RxBlockLenght = 0;
uint32_t tx_len = 0;
uint8_t rxdata[260];
uint8_t txdata[260];
__IO uint32_t block_rx_timeout_max_ms = 0;
__IO uint32_t last_bwt_count = 0;
__IO uint32_t NumberOfBytesEpilogue = 0;
__IO uint8_t conv;
uint8_t ppsn = 0;
__IO uint8_t pps_index=0;
__IO uint32_t PPS_timeout_max_ms = 0;
__IO uint32_t atr_byte_timeout;
__IO uint32_t tx_complete_flag = 0;
__IO uint8_t Block_TimeOut_IT_Occured = 0x00;

/* Private function prototypes -----------------------------------------------*/
static int32_t T1_ReceiveBlock(uint32_t *rx_length, uint8_t *rx_buffer);
static void T1_TransmitBlock(t1_TypeDef* t1, uint8_t* tx_buffer, uint32_t tx_length);
static uint8_t USART_Get_Byte(void);
static void USART_Send_Byte(uint8_t DataByte);
static uint32_t T1_ComputeBlockTimout(t1_TypeDef * t1);
static uint8_t ATR_ByteReceive(uint8_t *Data, uint32_t TimeOut);
static uint32_t reverse(uint32_t value);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  configure the USART and its baudrate. The baudrate is automatically
  *         selected bye the driver with a maximum defined by the user in t1_hal.h
  *         file (SC_CLK_MAX).
  * @param  etu_us: the etu value in us.
  * @param  baud: the working baudrate.
  * @retval The Smart card clock selected by the driver.
  */
uint32_t SC_USARTConfig(uint32_t* etu_us, uint32_t* baud)
{
  LL_USART_ClockInitTypeDef USART_ClockInitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  uint8_t usart_presc = 0;
  uint32_t pclk_frequency = 0;
  __IO uint32_t SC_Clk;
  uint32_t etu, baudrate;
  
  pclk_frequency = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_USART1);
  
  for (usart_presc=1;usart_presc<32;usart_presc++)
  {
    SC_Clk = pclk_frequency / (usart_presc*2);
    
    if(SC_Clk <= SC_CLK_MAX)
    {
      break;
    }
  }
  
  Compute_etu_baudrate(372, 1,SC_Clk, &etu, &baudrate);
  
  *etu_us = etu;
  *baud = baudrate;
  
  /* Enable USART clock */
//  SC_USART_APBPERIPHCLOCK(SC_USART_CLK, ENABLE);
  __HAL_RCC_USART1_CLK_ENABLE();//  SC_USART_APBPERIPHCLOCK(SC_USART_CLK, ENABLE);
  
  /* Enable USART IRQ */
  NVIC_InitStructure.NVIC_IRQChannel = SC_USART_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  
  /* SC_USART configuration ----------------------------------------------------*/
  /* SC_USART configured as follow:
  - Word Length = 9 Bits
  - 0.5 Stop Bit
  - Even parity
  - BaudRate = 9677 baud
  - Hardware flow control disabled (RTS and CTS signals)
  - Tx and Rx enabled
  - USART Clock enabled
  */
  
  /* USART Clock set to 4 MHz (PCLK2 (48 MHz) / 12) */
  LL_USART_SetSmartcardPrescaler(SC_USART, usart_presc);
  
  /* USART Guard Time set to 16 Bit */
  LL_USART_SetSmartcardGuardTime(SC_USART, 16);
  
  USART_ClockInitStructure.ClockOutput = LL_USART_CLOCK_ENABLE;
  USART_ClockInitStructure.ClockPolarity = LL_USART_POLARITY_LOW;
  USART_ClockInitStructure.ClockPhase = LL_USART_PHASE_1EDGE;
  USART_ClockInitStructure.LastBitClockPulse = LL_USART_LASTCLKPULSE_OUTPUT;
  LL_USART_ClockInit(SC_USART, &USART_ClockInitStructure);
  
  /* Configure the USART baudrate */
  SC_USART_Baud_Config(baudrate);
  
  /* Disable the NACK Transmission */
  LL_USART_DisableSmartcardNACK(SC_USART);
  
  /* Enable the Smartcard Interface */
  LL_USART_EnableSmartcard(SC_USART);
  
  /* Enable SC_USART */
  LL_USART_Enable(SC_USART);
  
  return SC_Clk;
}


/**
  * @brief  handle the different Transmit/receive interrupts for T=1 protocol.
  * @param  None.
  * @retval None.
  */
void T1_USART_TxRx_Handler(void)
{
  if(((LL_USART_IsEnabledIT_RXNE(SC_USART))&& (LL_USART_IsActiveFlag_RXNE(SC_USART))) != RESET)
  {
    rxdata[DataRxIndex++] = USART_Get_Byte();
    if (DataRxIndex == 0x03)
    {
      /* Set Block length for the reception */
      LL_USART_SetBlockLength(SC_USART, rxdata[2]);
    }
  }
  /* Block Time out*/
  if (((LL_USART_IsEnabledIT_RTO(SC_USART))&& (LL_USART_IsActiveFlag_RTO(SC_USART))) == SET)
  { 
    Block_TimeOut_IT_Occured++;
    LL_USART_ClearFlag_RTO(SC_USART);
    
    /* Disable The Receiver Time Out Interrupt */ 
    LL_USART_DisableIT_RTO(SC_USART);
    
  }
  /* End of block Length */
  if (((LL_USART_IsEnabledIT_EOB(SC_USART))&& (LL_USART_IsActiveFlag_EOB(SC_USART))) == SET)
  { 
    BlockReceiveDoneFlag++;
    LL_USART_ClearFlag_EOB(SC_USART);
    /* Disable The Receiver Time Out Interrupt */ 
    LL_USART_DisableIT_RTO(SC_USART);
  }
}

/**
  * @brief  Transmit T=1 block via USART.
  * @param  t1: a pointer to t1 stucture of T=1 protocol.
  * @param  tx_buffer: a pointer to the buffer of the block to be transmitted.
  * @param  tx_length: the lenght of the block to be transmitted.
  * @retval None.
  */
static void T1_TransmitBlock(t1_TypeDef * t1, uint8_t *tx_buffer, uint32_t tx_length)
{
  
  TxBufferIndex = 0;
  
  memcpy(txdata, tx_buffer, tx_length);
 
  /* Initialize transmission convention */
  conv = t1->convention;
  
  /* Disable SC_USART RXNE Interrupts */
  LL_USART_DisableIT_RXNE(SC_USART);
  
  /* Send the Block */
  while(TxBufferIndex < tx_length)
  { 
    /* waiting the transmit data register empty */
    while(LL_USART_IsActiveFlag_TXE(SC_USART) == RESET)
    {}
    USART_Send_Byte(tx_buffer[TxBufferIndex++]);
  }  
  
  /* waiting the end of transfert */
  while(LL_USART_IsActiveFlag_TC(SC_USART) == RESET)
  {}
  
  /* Flush the SC_USART DR */
  (void)LL_USART_ReceiveData9(SC_USART);
  
  /* Clear the Over run flag */
  LL_USART_ClearFlag_ORE(SC_USART);
  
  /* Enable SC_USART RXNE and PE Interrupts */
  LL_USART_EnableIT_RXNE(SC_USART);
  
  tx_complete_flag = 1;
  
  /* Reset the index of the Rx buffer */
  DataRxIndex = 0;
  
  BlockReceiveDoneFlag = 0;
  
  pps_exchange_flag = 0;
  
  /* Initialize the number of bytes in the epilogue field */
  NumberOfBytesEpilogue = t1->rc_bytes;
  
  /* Start the counter for Block waiting time (the timeout of the card response)  */
  /* - BWT_ms: Value calculated from BWI (TBi i>2) and then multiplied by WTX requested by the card
     - CWT_ms: Value calculated from CWI (TBi i>2) */
  
  block_rx_timeout_max_ms = T1_ComputeBlockTimout(t1);
  
  /* Reset to default value after the transmit */
  t1->wtx = 1;
}


/**
  * @brief  Receive T=1 block via USART with timeout.
  * @param  t1: a pointer to t1 stucture of T=1 protocol.
  * @param  rx_length: a pointer to the length of the received block.
  * @param  rx_buffer: a pointer to the received block buffer.
  * @retval the reception status of the received block. The returned value can
  *   be one of the following:
  *     - T1_RECEIVE_SUCCESS: the block was received successfully.
  *     - T1_BWT_TIMEOUT: Block timeout error.
  */
static int32_t T1_ReceiveBlock(uint32_t* rx_length, uint8_t *rx_buffer)
{
  
  /* Clear the Overrun flag */
  LL_USART_ClearFlag_ORE(SC_USART);
  
  /* Clear the end of Block flag */
  LL_USART_ClearFlag_EOB(SC_USART);
  
  /* Set Block length for the reception (default value 0xFF)*/
  LL_USART_SetBlockLength(SC_USART, 0xFF);
  
  /* Enable SC_USART EOB Interrupt */
  LL_USART_EnableIT_EOB(SC_USART);
  
  /* Enable the receiver Time out */
  LL_USART_DisableRxTimeout(SC_USART);
  
  /* Configure the Block Receiver Time Out taking in acount that the guardTime < t <Receiver time out */
  LL_USART_SetRxTimeout(SC_USART, Bwt_us);
  
  /* Enable the receiver time out Interrupt */
  LL_USART_EnableIT_RTO(SC_USART);
  
  /* Enable the receiver time out */
  LL_USART_EnableRxTimeout(SC_USART);
  
  /* Waiting the First Character or Time out*/ 
  while ((DataRxIndex == 0) && (Block_TimeOut_IT_Occured == 0))
  {}
  
  /* Receive block timeout occured ? (the RTO counter reaches 0) */
  if(Block_TimeOut_IT_Occured == 1)
  {
    /* Reset the index of the Rx buffer */
    DataRxIndex = 0;
    
    /* Disable SC_USART RXNE Interrupt */
    LL_USART_DisableIT_RXNE(SC_USART);
    
    /* Return Block waiting time timout */
    return T1_BWT_TIMEOUT;	
  }
  /* First Data received */
  else
  {
    /* Disable the receiver Time out */
    LL_USART_DisableRxTimeout(SC_USART);
    
    /* Configure the character Receiver Time Out */
    LL_USART_SetRxTimeout(SC_USART, Cwt_us);
    
    /* Enable again the receiver Time out */
    LL_USART_EnableRxTimeout(SC_USART);
    
    /* Waiting the First Character or Time out*/ 
    while ((BlockReceiveDoneFlag == 0) && (Block_TimeOut_IT_Occured == 0))
    {}
    
    memcpy(rx_buffer, rxdata, DataRxIndex);
    
    /* Get the lenght of the received block */
    *rx_length = DataRxIndex;
    
    /* Reset the index of the Rx buffer */
    DataRxIndex = 0;
    
    /* Reset the Rx block lenght */
    RxBlockLenght = 0;
    
    if (BlockReceiveDoneFlag != 0x00)
    {     
      /* Reset the flag of the Rx block */
      BlockReceiveDoneFlag = 0;
      
      /* Return success of the receive */
      return T1_RECEIVE_SUCCESS;
    }
    else
    {
      /* Return Character waiting time timout */
      return T1_BWT_TIMEOUT;
    }
  }
}

/**
  * @brief  Tranceive T=1 blocks.
  * @param  t1: a pointer to t1 stucture of T=1 protocol.
  * @param  txrxBuffer: a pointer to Transmit/Receive buffer.
  * @param  tx_length: the length of the block to be tranmitted.
  * @retval the reception status of the received block. The returned value can
  *   be one of the following:
  *     - (>0): the length of the received block if the reception has succeeded.
  *     - (-1): in case of BWT timeout.
  *     - (-2): in case of parity error.
  *     - (-3): block malformed.
  */
int32_t T1_TxRxBlock(t1_TypeDef * t1, uint8_t *txrxBuffer, uint32_t tx_length)
{
  int32_t  RxStatus = 0;
  uint32_t RxLen = 0;
  uint32_t Rxlength = 0;
  
  /* Send a block to the card */
  T1_TransmitBlock(t1, txrxBuffer, tx_length);
  
  /* Get the block response from the card */
  RxStatus = T1_ReceiveBlock(&RxLen, txrxBuffer);
  
  if (RxStatus == T1_PARITY_ERROR)
  {
    /* Receive Error: parity error */
    return -2;
  }
  
  if (RxStatus == T1_BWT_TIMEOUT)
  {
    
    /* Receive Error: timeout error */
    return -1;
  }
  
  /* 3: the length of Prologue field */
  Rxlength = 3 + txrxBuffer[2] + t1->rc_bytes;
  
  if (RxLen != Rxlength)
  {
    /*Block received malformed*/
    return -3;
  }
  
  return RxLen;
}

/**
  * @brief  Decrements the different timeout variables for T=1 protocol.
  * @param  None
  * @retval None
  */
void T1_TimingDecrement(void)
{
  if (block_rx_timeout_max_ms != 0x00)
  {
     block_rx_timeout_max_ms--;
  }
  if (PPS_timeout_max_ms != 0x00)
  {
     PPS_timeout_max_ms--;
  }
  if(atr_byte_timeout != 0x00)
  {
     atr_byte_timeout--;
  }
}

/**
  * @brief  Get byte from USART in direct or inverse convension.
  * @param  None.
  * @retval the received data byte.
  */
static uint8_t USART_Get_Byte(void)
{
   uint8_t DataByte = 0;

   DataByte = (uint8_t)LL_USART_ReceiveData9(SC_USART);

   if(conv == DIRECT_CONV)
   {
      return DataByte;
   }
   else
   {
      /* Rotate the byte and inverse the bits */
      return (uint8_t)(~(reverse(DataByte) >> 24));
   }
}


/**
  * @brief  Send byte via USART in direct or inverse convension.
  * @param  DataByte: the data byte to be transmitted.
  * @retval None.
  */
static void USART_Send_Byte(uint8_t DataByte)
{
   uint8_t temptxdata = 0;

   if(conv == DIRECT_CONV)
   {
      LL_USART_TransmitData9(SC_USART, DataByte);
   }
   else
   {
      /* Rotate the byte and inverse its bits */
      temptxdata =  (uint8_t)(~(reverse(DataByte) >> 24));

      LL_USART_TransmitData9(SC_USART, temptxdata);
   }
}

/**
  * @brief  Transmit PPS request via USART.
  * @param  request: a pointer to the PPS request buffer.
  * @param  len_request: the lenght of the PPS request buffer.
  * @param  etu_usec: etu in us.
  * @retval None
  */
void PPS_Transmit(uint8_t *request, uint8_t len_request, uint32_t etu_usec)
{
  
  TxBufferIndex = 0;
  
  memcpy(txdata, request, len_request);
  
  tx_len = len_request;
  
  
  /* Enable SC_USART RXNE and PE Interrupts */
  LL_USART_DisableIT_RXNE(SC_USART);
  
  /* Enable SC_USART TC Interrupt */
  LL_USART_EnableIT_TC(SC_USART);
  
  while(tx_complete_flag==0)
  {
  }
  
  tx_complete_flag=0;
  
  /* Reset the index of the Rx buffer */
  DataRxIndex = 0;
  
  BlockReceiveDoneFlag = 0;
  
  pps_exchange_flag = 1;
  
  DataRxIndex = 0;
  
  ppsn = 0;
  
  /* Enable SC_USART RXNE and PE Interrupts */
  LL_USART_EnableIT_RXNE(SC_USART);
  
  PPS_timeout_max_ms = (6*9600*etu_usec)/1000;
}

/**
  * @brief  Receive PPS response via USART.
  * @param  response: a pointer to the PPS response buffer.
  * @param  len_response: a pointer to a variable that contains the lenght of
  *         the PPS response buffer.
  * @param  etu_usec: etu in us.
  * @retval None.
  */
int32_t PPS_Receive(uint8_t *response, uint8_t *len_response)
{
  
  while((BlockReceiveDoneFlag == 0) && (PPS_timeout_max_ms != 0))
  {
  }
  
  /* Clear PPS exchange flag */
  pps_exchange_flag = 0;
  
  /* Receive block timeout occured ? (the block countdown counter reaches 0) */
  if(PPS_timeout_max_ms == 0)
  {
    /* Disable SC_USART RXNE Interrupt */
    LL_USART_DisableIT_RXNE(SC_USART);
    
    DataRxIndex = 0;
    
    /* Return response timout */
    return T1_PPS_TIMEOUT;
  }
  
  /* Parity errors occured ? */
  if(ParityErrorCount != 0)
  {
    /* Reset the Parity error flag */
    ParityErrorCount = 0;
    
    DataRxIndex = 0;
    
    /* Disable SC_USART RXNE and PE Interrupts */
    LL_USART_DisableIT_RXNE(SC_USART);
    
    /* Return parity error */
    return T1_PARITY_ERROR;
  }
  
  memcpy(response, rxdata, DataRxIndex);
  
  *len_response = (uint8_t)DataRxIndex;
  
  DataRxIndex = 0;
  
  /* Return success of the receive */
  return T1_RECEIVE_SUCCESS;
}

/**
  * @brief  Compute the block timeout of the longest block.
  * @param  t1: a pointer to t1 stucture of T=1 protocol.
  * @retval timout_ms: the block timeout of the longest block.
  */
static uint32_t T1_ComputeBlockTimout(t1_TypeDef * t1)
{
  uint32_t timout_ms = 0;
  uint32_t bwt_us = 0, cwt_us = 0;
  
  bwt_us = 11 * t1->etu_us +(uint32_t)(((long long)(1<<t1->bwi)* K_BWT) / t1->frequency);
  Bwt_us = bwt_us; 
  
  /* Character waiting time */
  cwt_us = (uint32_t)(11 + (long long)(1<<t1->cwi)) * t1->etu_us;
  Cwt_us = cwt_us;
  
  /* The maximum timing that the block can take to be received */
  timout_ms = (uint32_t)(((258*cwt_us)+ (bwt_us * t1->wtx))/1000);
  
  return (timout_ms + 1);
}

/**
  * @brief  Computes etu (elementatry time unit) in us as well as the baudrate.
  * @param  F: transmission factor F.
  * @param  D: transmission factor D.
  * @param  frequency: smartcard clock frequency.
  * @param  etu: a pointer to a variable that contains the etu value in us.
  * @param  baudrate: a pointer to a variable that contains baudrate value.
  * @retval None.
  */
void Compute_etu_baudrate(uint32_t F, uint32_t D, uint32_t frequency, uint32_t* etu, uint32_t *baudrate)
{
  uint32_t temp_etu;
  uint32_t temp_baudrate;
  
  temp_baudrate = (D * frequency)/F;
  
  temp_etu = 1000000/temp_baudrate;
  
  *baudrate = temp_baudrate;
  *etu = temp_etu+1;
}

/**
  * @brief  Get the answer to reset frame from the smartcard.
  * @param  ATR_buffer: a pointer to a buffer that contains the ATR frame.
  * @param  etu_us: the etu value in us.
  * @param  max_length: the maximum length estimated of the ATR frame.
  * @retval None.
  */
void SC_GetATR(uint8_t *ATR_buffer, uint32_t etu_us, uint32_t max_length)
{
  uint8_t data = 0;
  uint32_t i = 0;
  
  /* Initial waiting time in ms (see ISO7816-3 6.3.2 character frame) */
  uint32_t byte_timeout = (9600*etu_us)/1000;
  
  /* Set RSTIN HIGH */
  SC_Reset(1);
  
  /* Check response with reset low*/
  for (i = 0; i < max_length; i++)
  {
    if((ATR_ByteReceive(&data, byte_timeout)) == 0)
    {
      /* Read the data while no timeout occured */
      ATR_buffer[i] = data;
    }
  }
  
  if (ATR_buffer[0] == 0x03) /* Inverse convention 0x3F is 0x03 in inverse convention*/
  {
    /* invert all ATR buffer */
    for(i=0; i<max_length; i++)
    {
      ATR_buffer[i] = (uint8_t)(~(reverse(ATR_buffer[i]) >> 24));
    }
    
    /* Clear PE flag by writing 1 to PECF bit in the USART_ISR register*/
    SC_USART->ICR |= USART_ICR_PECF;
    
    
    SC_USART->CR1 |= 0x200; /* Switch to Odd parity for inverse convention */
  }
}

/**
  * @brief  Receive ATR byte from the smart card via USART.
  * @param  Data: a pointer to a variable that contains the received byte.
  * @param  TimeOut: the byte timeout in us.
  * @retval the status of the byte reception. The returned value can
  *   be one of the following:
  *     - 0 : an ATR byte is received without timeout.
  *     - 1 : ATR byte timeout occured.
  */
static uint8_t ATR_ByteReceive(uint8_t *Data, uint32_t TimeOut)
{
  
  atr_byte_timeout = TimeOut;
  
  while((LL_USART_IsActiveFlag_RXNE(SC_USART) == RESET) && (atr_byte_timeout != 0))
  {
  }
  
  if(atr_byte_timeout != 0)
  {
    *Data = (uint8_t)LL_USART_ReceiveData9(SC_USART);
    return 0;
  }
  else
  {
    return 1;
  }
}

/**
  * @brief  Initialize the Smartcard hardware for smartcard application.
  * @param  None.
  * @retval None.
  */
void SC_HardwareInit(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStructure;
  LL_EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Setup SysTick Timer to interrupt each 1 msec for smart card time-base timings */
  SysTick_Config(SystemCoreClock/1000);
  
  /* Enable GPIO clocks */
  LL_AHB1_GRP1_EnableClock(SC_USART_TX_GPIO_CLK | SC_3_5V_GPIO_CLK | SC_RESET_GPIO_CLK | SC_CMDVCC_GPIO_CLK);
  
  /* Enable SYSCFG clock */
//  RCC_APB2PeriphClockCmd(LL_APB1_GRP2_PERIPH_SYSCFG, ENABLE);
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  
  /* Connect PXx to SC_USART_TX */
//  GPIO_PinAFConfig(SC_USART_TX_GPIO_PORT, SC_USART_TX_SOURCE, SC_USART_TX_AF);
  
  /* Connect PXx to SC_USART_CK */
//  GPIO_PinAFConfig(SC_USART_CK_GPIO_PORT, SC_USART_CK_SOURCE, SC_USART_CK_AF);
  
  /* Configure USART CK pin as alternate function push-pull */
  GPIO_InitStructure.Pin = SC_USART_CK_PIN;
  GPIO_InitStructure.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStructure.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(SC_USART_CK_GPIO_PORT, &GPIO_InitStructure);
  
  /* Configure USART Tx pin as alternate function open-drain */
  GPIO_InitStructure.Pin = SC_USART_TX_PIN;
  GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStructure.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(SC_USART_TX_GPIO_PORT, &GPIO_InitStructure);
  
  /* Configure Smartcard CMDVCC pin */
  GPIO_InitStructure.Pin = SC_CMDVCC_PIN;
  GPIO_InitStructure.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL; 
  GPIO_InitStructure.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(SC_CMDVCC_GPIO_PORT, &GPIO_InitStructure);

  /* Configure Smartcard Reset pin */
  GPIO_InitStructure.Pin = SC_RESET_PIN;
  GPIO_InitStructure.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(SC_RESET_GPIO_PORT, &GPIO_InitStructure);

  /* Configure Smartcard 3/5V pin */
  GPIO_InitStructure.Pin = SC_3_5V_PIN;
  GPIO_InitStructure.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(SC_3_5V_GPIO_PORT, &GPIO_InitStructure);

  /* Select 5V */ 
  SC_VoltageConfig(SC_VOLTAGE_5V);

  /* Disable CMDVCC */
  SC_PowerCmd(DISABLE);

  /* Set RSTIN HIGH */  
  SC_Reset(1);
    						                                
  /* Configure Smartcard OFF pin */
  GPIO_InitStructure.Pin = SC_OFF_PIN;
  GPIO_InitStructure.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStructure.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStructure.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(SC_OFF_GPIO_PORT, &GPIO_InitStructure);
  
  /* Configure EXTI line connected to Smartcard OFF Pin */
  LL_SYSCFG_SetEXTISource(SC_OFF_EXTI_PORT_SOURCE, SC_OFF_EXTI_PIN_SOURCE);

  EXTI_ClearITPendingBit(SC_OFF_EXTI_LINE);
  
  EXTI_InitStructure.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStructure.Trigger = LL_EXTI_TRIGGER_RISING;
  EXTI_InitStructure.Line_0_31 = SC_OFF_EXTI_LINE;
  EXTI_InitStructure.LineCommand = ENABLE;
  LL_EXTI_Init(&EXTI_InitStructure);

  /* Clear the SC_OFF_EXTI_IRQn Pending Bit */
  NVIC_ClearPendingIRQ(SC_OFF_EXTI_IRQn);
  NVIC_InitStructure.NVIC_IRQChannel = SC_OFF_EXTI_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);    
  
}

/**
  * @brief  Configure the USART baudrate for smartcard application.
  * @param  baud: the USART baudrate.
  * @retval None.
  */
void SC_USART_Baud_Config(uint32_t baud)
{
  LL_USART_InitTypeDef USART_InitStructure;

  USART_InitStructure.BaudRate = baud;
  USART_InitStructure.DataWidth = LL_USART_DATAWIDTH_9B;
  USART_InitStructure.StopBits = LL_USART_STOPBITS_1_5;
  USART_InitStructure.Parity = LL_USART_PARITY_EVEN;
  USART_InitStructure.TransferDirection = LL_USART_DIRECTION_RX | LL_USART_DIRECTION_TX;
  USART_InitStructure.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStructure.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(SC_USART, &USART_InitStructure); 
}

/**
  * @brief  Enables or disables the power to the Smartcard.
  * @param  NewState: new state of the Smartcard power supply. 
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void SC_PowerCmd(FunctionalState NewState)
{
  if(NewState != DISABLE)
  {
    LL_GPIO_ResetOutputPin(SC_CMDVCC_GPIO_PORT, SC_CMDVCC_PIN);
  }
  else
  {
    LL_GPIO_SetOutputPin(SC_CMDVCC_GPIO_PORT, SC_CMDVCC_PIN);
  } 
}

/**
  * @brief  Sets or clears the Smartcard reset pin.
  * @param  ResetState: specifies the state of the Smartcard reset pin.
  *   This parameter can be one of the following values:
  *     @arg Bit_RESET: Set RST pin of ST8024.
  *     @arg Bit_SET: Reset RST pin of ST8024.
  * @retval None.
  */
void SC_Reset(BitAction ResetState)
{
  HAL_GPIO_WritePin(SC_RESET_GPIO_PORT, SC_RESET_PIN, ResetState);
}

/**
  * @brief  Configures the card power voltage.
  * @param  SC_Voltage: specifies the card power voltage.
  *   This parameter can be one of the following values:
  *     @arg SC_Voltage_5V: 5V cards.
  *     @arg SC_Voltage_3V: 3V cards.
  * @retval None.
  */
void SC_VoltageConfig(uint32_t SC_Voltage)
{
  if(SC_Voltage == SC_Voltage_5V)
  {
    /* Select Smartcard 5V */
    LL_GPIO_SetOutputPin(SC_3_5V_GPIO_PORT, SC_3_5V_PIN);
  }
  else
  {
    /* Select Smartcard 3V */
    LL_GPIO_ResetOutputPin(SC_3_5V_GPIO_PORT, SC_3_5V_PIN);
  }
}

/**
  * @brief  Reverse bit order of value.
  * @param  value: specifies the card power voltage.
  * @retval reversed value.
  */
static uint32_t reverse(uint32_t value) 
{
  uint32_t h = 0;
  uint32_t i = 0;
  
  for(h = i = 0; i < 32; i++)
  {
    h = (h << 1) + (value & 1); 
    value >>= 1; 
  }
  
  return h;
}

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
