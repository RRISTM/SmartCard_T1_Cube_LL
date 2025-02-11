/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "t1_hal.h"
#include "t1_protocol.h"
#include "t1_protocol_param.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Modifiable parameters */
#define SAD           0x0  /* Source address: reader (allowed values 0 -> 7) */
#define DAD           0x0  /* Destination address: card (allowed values 0 -> 7) */
#define IFSD_VALUE    254  /* Max length of INF field Supported by the reader */
#define SC_FILE_SIZE  0x100   /* File size */
#define SC_FILE_ID    0x0001  /* File identifier */
#define SC_CLASS      0x00

/* Constant parameters */
#define INS_SELECT_FILE    0xA4 /* Select file instruction */
#define INS_READ_FILE      0xB0 /* Read file instruction */
#define INS_WRITE_FILE     0xD6 /* Write file instruction */
#define TRAILER_LENGTH     2    /* Trailer lenght (SW1 and SW2: 2 bytes) */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define NAD    (((DAD&0x7)<<4) | (SAD&0x7)) /* Node Address byte */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

RTC_HandleTypeDef hrtc;

/* USER CODE BEGIN PV */
ATR_TypeDef atr; /* Answer To Reset structure */
t1_TypeDef T1;   /* T=1 protocol structure */
__IO int32_t ResponseStatus = 0; /* Communication Response status */
uint8_t C_APDU[300];  /* APDU Command buffer */
uint8_t R_APDU[300];  /* APDU Response buffer */
uint8_t ATR_buf[ATR_MAX_SIZE]; /* Answer To Reset buffer */
uint32_t CardClkFreq = 0;  /* The Smartcard clock frequency in Hz */
uint32_t etu_usec = 0; /* Elementary Time Unit in microsecond */
uint32_t SC_baud = 0;  /* SmartCard baudrate */
int8_t protocol= -1;   /* Default protocol (initialized to -1) */
uint32_t apdu_index;   /* Index of the APDU buffer */
uint8_t atr_status;    /* The status of the ATR response */
__IO uint32_t TimingDelay = 0; 
__IO uint32_t CardInserted = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_SMARTCARD_Init(void);
static void MX_CRC_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_SMARTCARD_Init();
  MX_CRC_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  /* Loop while no Smartcard is detected */  
//  while(CardInserted == 0)
//  {
//  }

  /* Insert delay of 100ms for signal stabilization */
  HAL_Delay(100);
    
  /* Configure the USART for SmartCard application and get the selected frequency */
  CardClkFreq = SC_USARTConfig(&etu_usec,&SC_baud);
  
  /* Enable CMDVCC */
  SC_PowerCmd(ENABLE);
  
  /* Reset the card */
  SC_Reset(0);
  
  
  /************************* Answer To Reset (ATR) ******************************/ 
  
  /* ---ATR request...--- */
  
  /* Get the answer to reset (ATR) frame from the card */    
  SC_GetATR(ATR_buf, etu_usec, ATR_MAX_SIZE);
  
  /* Decode the ATR frame */
  atr_status = ATR_Decode(&atr, &ATR_buf[0], ATR_MAX_SIZE);
  
  /* If the ATR is malformed */
  if(atr_status != 0)
  {
    /* ---ATR ERROR--- */
    while (1)
    {
      /* The user can reset the card at this stage */
    }
  }
  else
  {
    
    /* ---ATR OK--- */
    /* ---ATR received: ATR_buf, atr.length--- */
    
  }
  
  /* ---SmartCard Clk: CardClkFreq--- */
  /* ---SmartCard baudrate: SC_baud--- */
  
  
  /* Get the default protocol */
  ATR_GetDefaultProtocol(&atr, &protocol);
  
  /* If the protocol used by the card is T=1, start the demo */
  if(protocol == ATR_PROTOCOL_TYPE_T1)
  {
    
    /********************** T=1 Protocol initialization ****************************/ 
    
    /* Init the protocol structure */
    T1_Protocol_Init(&T1,CardClkFreq);
    
    /* Set F and D parameters and get the new etu value in micro-seconds */
    etu_usec = Set_F_D_parameters(&atr,CardClkFreq);
    
    /* Set the etu to be used by the protocol */
    (void)T1_SetParameter(&T1, T1_PROTOCOL_ETU, etu_usec);
    
    /* Set the convention of the protocol */
    Set_Convention(&atr,&T1);
    
    /* Set and configure the extra guardtime value */
    Set_EGT(&atr); 
    
    /* Set the IFSC (card IFS) value */
    Set_IFSC(&atr, &T1);
    
    /* Set the CWT and BWT values */
    Set_CWT_BWT(&atr, &T1);      
    
    /* Set the EDC type (LRC or CRC) */
    Set_EDC(&atr, &T1); 
    
    /*************************** Start T1 Protocol ********************************/ 
    
    /*------- Send IFSD request --------------------------------------------------*/     
    
    /* Negotiate IFSD: we indicate to the card a new IFSD that the reader can support */
    ResponseStatus = T1_Negotiate_IFSD(&T1, NAD, IFSD_VALUE);
    
    /* If the IFSD request communication has failed */
    if(ResponseStatus<0)
    {
      /* ---IFSD communication error--- */
      while (1)
      {
        /* The user can reset the card at this stage */
      }
    }
    
    
    /*------- Send APDU: Select File Command -------------------------------------*/     
    
    /* Send Select File Command */  
    C_APDU[0] = SC_CLASS;                    /* CLA */
    C_APDU[1] = INS_SELECT_FILE;             /* INS: Select File */
    C_APDU[2] = 0x00;                        /* P1 */
    C_APDU[3] = 0x00;                        /* P2 */
    C_APDU[4] = 0x02;                        /* Lc */
    C_APDU[5] = (uint8_t)(SC_FILE_ID>>8);    /* Data 1 */
    C_APDU[6] = (uint8_t)(SC_FILE_ID&0xFF);  /* Data 2 */
    
    /* ---Sending APDU:  C_APDU, 7--- */
    
    /* Send/Receive APDU command/response: Select File having ID = SC_FILE_ID */
    ResponseStatus = T1_APDU(&T1, NAD, C_APDU, 7, R_APDU, TRAILER_LENGTH);
    
    /* If the APDU communication has failed */
    if(ResponseStatus<0)
    {
      /* ---APDU communication error--- */
      while (1)
      {
        /* The user can reset the card at this stage */
      }
    }
    else
    {
      /* ---APDU response: R_APDU, ResponseStatus--- */    
    }
    
    
    /*------- Send APDU: Read File Command ---------------------------------------*/     
    
    /* Select file */
    C_APDU[0] = SC_CLASS;       /* CLA */
    C_APDU[1] = INS_READ_FILE;  /* INS: Read File */
    C_APDU[2] = 0x00;           /* P1 */
    C_APDU[3] = 0x00;           /* P2 */
    C_APDU[4] = 0x00;           /* Lc: read 256 bytes */
    
    /* ---Sending APDU: C_APDU, 5--- */ 
    
    /* Send/Receive APDU command/response: Read File (256 bytes) */
    ResponseStatus = T1_APDU(&T1, NAD, C_APDU, 5, R_APDU, 256+TRAILER_LENGTH);
    
    /* If the APDU communication has failed */
    if(ResponseStatus<0)
    {
      /* ---APDU communication error--- */
      while (1)
      {
        /* The user can reset the card at this stage */
      }
    }
    else
    { 
      /* ---APDU response: R_APDU, ResponseStatus--- */ 
    }
    
    
    /*------- Send APDU: Write File Command --------------------------------------*/      
    
    /* Select file */
    C_APDU[0] = SC_CLASS;        /* CLA */
    C_APDU[1] = INS_WRITE_FILE;  /* INS: Write File */
    C_APDU[2] = 0x00;            /* P1 */
    C_APDU[3] = 0x00;            /* P2 */
    C_APDU[4] = 0xFF;            /* Lc */
    
    apdu_index = 5;
    
    while(apdu_index<261)
    {
      /* initialize the APDU buffer (data field) */
      C_APDU[apdu_index] = apdu_index-5;
      apdu_index++;
    }
    
    /* ---Sending APDU--- */
    
    /* Send/Receive APDU command/response: Write File 256 bytes */
    ResponseStatus = T1_APDU(&T1, NAD, C_APDU, 261, R_APDU, TRAILER_LENGTH);
    
    /* If the APDU communication has failed */
    if(ResponseStatus<0)
    {   
      /* ---APDU communication error--- */
      while (1)
      {
        /* The user can reset the card at this stage */
      }
    }
    else
    {     
      /* ---APDU response: R_APDU, ResponseStatus--- */     
    }
    
    
    /*------- Send APDU: Read File Command ---------------------------------------*/     
    
    /* Select file */
    C_APDU[0] = SC_CLASS;       /* CLA */
    C_APDU[1] = INS_READ_FILE;  /* INS: Read File */
    C_APDU[2] = 0x00;           /* P1 */
    C_APDU[3] = 0x00;           /* P2 */
    C_APDU[4] = 0x00;           /* Lc */
    
    /* ---Sending APDU:  C_APDU, 5--- */ 
    
    /* Send/Receive APDU command/response: Read File (256 bytes) */
    ResponseStatus = T1_APDU(&T1, NAD, C_APDU, 5, R_APDU, 256+TRAILER_LENGTH);
    
    /* If the APDU communication has failed */
    if(ResponseStatus<0)
    {
      /* ---APDU communication error--- */
      while (1)
      {
        /* The user can reset the card at this stage */
      }
    }
    else
    {   
      /* ---APDU response: R_APDU, ResponseStatus--- */ 
    }
    
    
    /*------- Send APDU: Write File Command --------------------------------------*/        
    
    /* Select file */
    C_APDU[0] = SC_CLASS;        /* CLA */
    C_APDU[1] = INS_WRITE_FILE;  /* INS: Write File */
    C_APDU[2] = 0x00;            /* P1 */
    C_APDU[3] = 0x00;            /* P2 */
    C_APDU[4] = 0xFF;            /* Lc */
    
    apdu_index = 5;
    
    while(apdu_index<261)
    {
      /* Get and invesre the file received from the card */
      C_APDU[apdu_index] = ~(R_APDU[apdu_index-5]);
      apdu_index++;
    }
    
    /* ---Sending APDU: ", C_APDU, 261--- */
    
    /* Send/Receive APDU command/response: Write File (256 bytes) */
    ResponseStatus =  T1_APDU(&T1, NAD, C_APDU, 261, R_APDU, TRAILER_LENGTH);
    
    /* If the APDU communication has failed */
    if(ResponseStatus<0)
    {  
      /* ---APDU communication error--- */
      while (1)
      {
        /* The user can reset the card at this stage */
      }
    }
    else
    {
      /* ---APDU response: R_APDU, ResponseStatus--- */
    }
    
    /*------- Send APDU: Read File Command -------------------------------------*/     
    
    /* Select file */
    C_APDU[0] = SC_CLASS;       /* CLA */
    C_APDU[1] = INS_READ_FILE;  /* INS: Read File */
    C_APDU[2] = 0x00;           /* P1 */
    C_APDU[3] = 0x00;           /* P2 */
    C_APDU[4] = 0x00;           /* Lc */
    
    /* ---Sending APDU: C_APDU, 5--- */
    
    /* Send/Receive APDU command/response: Read File (256 bytes) */
    ResponseStatus =  T1_APDU(&T1, NAD, C_APDU, 5, R_APDU, 256+TRAILER_LENGTH);
    
    /* If the APDU communication has failed */
    if(ResponseStatus<0)
    {
      /* ---APDU communication error--- */
      while (1)
      {
        /* The user can reset the card at this stage */
      }
    }
    else
    { 
      /* ---APDU response:  R_APDU, ResponseStatus--- */ 
      /* ---All operations were executed successfully --- */
      // STM_EVAL_LEDOn(LED1);
      // STM_EVAL_LEDOn(LED2);
      // STM_EVAL_LEDOn(LED3);
      // STM_EVAL_LEDOn(LED4);
       __NOP();
    }     
  }
  else
  {
    /* ---SmartCard is not compatible with T=1 protocol--- */
    // STM_EVAL_LEDOff(LED1);
    // STM_EVAL_LEDOn(LED2);
    // STM_EVAL_LEDOff(LED3);
    // STM_EVAL_LEDOff(LED4);
     __NOP();
  }

  /* ----*-*-*- DEMO END -*-*-*---- */   
  CardInserted = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
//  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_LSI_Enable();

   /* Wait till LSI is ready */
  while(LL_RCC_LSI_IsReady() != 1)
  {

  }
  LL_PWR_EnableBkUpAccess();
  if(LL_RCC_GetRTCClockSource() != LL_RCC_RTC_CLKSOURCE_LSI)
  {
    LL_RCC_ForceBackupDomainReset();
    LL_RCC_ReleaseBackupDomainReset();
    LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSI);
  }
  LL_RCC_EnableRTC();
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_12);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(48000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK1);
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_SMARTCARD_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};
  LL_USART_ClockInitTypeDef USART_ClockInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration
  PA8   ------> USART1_CK
  PA9   ------> USART1_TX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART1 DMA Init */

  /* USART1_RX Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MDATAALIGN_BYTE);

  /* USART1_TX Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_BYTE);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, 0);
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 38400;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_9B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1_5;
  USART_InitStruct.Parity = LL_USART_PARITY_EVEN;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;

  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_SetSmartcardGuardTime(USART1, 0);
  LL_USART_SetBlockLength(USART1, 0);
  LL_USART_SetSmartcardAutoRetryCount(USART1, 0);
  LL_USART_SetSmartcardPrescaler(USART1, 10);
  LL_USART_EnableSmartcardNACK(USART1);
  USART_ClockInitStruct.ClockOutput = LL_USART_CLOCK_ENABLE;
  USART_ClockInitStruct.ClockPolarity = LL_USART_POLARITY_LOW;
  USART_ClockInitStruct.ClockPhase = LL_USART_PHASE_1EDGE;
  USART_ClockInitStruct.LastBitClockPulse = LL_USART_LASTCLKPULSE_NO_OUTPUT;
  LL_USART_ClockInit(USART1, &USART_ClockInitStruct);
  LL_USART_ConfigSmartcardMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0);
  NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, VCC_Pin|LD1_Pin|LD2_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(v35_GPIO_Port, v35_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : VCC_Pin LD1_Pin LD2_Pin LD3_Pin */
  GPIO_InitStruct.Pin = VCC_Pin|LD1_Pin|LD2_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RST_Pin */
  GPIO_InitStruct.Pin = RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OFF_Pin */
  GPIO_InitStruct.Pin = OFF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OFF_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : v35_Pin */
  GPIO_InitStruct.Pin = v35_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(v35_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD4_Pin */
  GPIO_InitStruct.Pin = LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD4_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
