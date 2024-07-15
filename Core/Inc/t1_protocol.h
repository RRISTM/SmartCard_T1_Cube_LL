/**
  ******************************************************************************
  * @file    Smartcard_T1/inc/t1_protocol.h 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    13-May-2013
  * @brief   This file contains all the functions prototypes for the T=1 protocol
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
#ifndef __T1_PROTOCOL_H
#define __T1_PROTOCOL_H

/* Includes ------------------------------------------------------------------*/
#include "legacy.h"
#include "ll_includes.h"
#include "stm32f0xx_hal_conf.h"
#include "buffer.h"

/* Exported constants --------------------------------------------------------*/
/* T=1 protocol constants */
#define T1_I_BLOCK        0x00  /* PCB (I-block: b8 = 0)  */
#define T1_R_BLOCK        0x80  /* PCB (R-block: b8 b7 = 10) */
#define T1_S_BLOCK        0xC0  /* PCB (S-block: b8 b7 = 11) */
#define T1_MORE_BLOCKS    0x20  /* PCB (b6): More data bit */
#define T1_BUFFER_SIZE    (3 + 254 + 2) /* Prologue + Information field + EDC */

/* Exported types ------------------------------------------------------------*/
enum {
 T1_PROTOCOL_RECV_TIMEOUT = 0,
 T1_PROTOCOL_CHECKSUM_CRC,
 T1_PROTOCOL_CHECKSUM_LRC,
 T1_PROTOCOL_IFSC,
 T1_PROTOCOL_IFSD,
 T1_PROTOCOL_STATE,
 T1_PROTOCOL_MORE,
 T1_PROTOCOL_CONVENTION,
 T1_PROTOCOL_ETU,
 T1_PROTOCOL_FREQUENCY,
 T1_PROTOCOL_BWI,
 T1_PROTOCOL_CWI
};

typedef struct {
 uint32_t   etu_us;    /* Logical Unit Number: see ifdhandler.c file line 150 for more details */
 uint32_t   frequency; /* Smart card frequency */
 uint32_t   cwi;       /* Character waiting time */
 uint32_t   bwi;       /* Block waiting time */
 uint32_t   wtx;       /* Waiting time extension value (extension of BWT: Block Waiting Time) */
 uint32_t   state;     /* State of the communication */
 uint8_t    ns;       /* reader side: N(S): the send-sequence number of the block  */
 uint8_t    nr;       /* card side: N(R): the number of the expected l-block */ 
 uint8_t    ifsc;      /* Maximum length of Information field which can be received by the card */ 
 uint8_t    ifsd;      /* Maximum length of Information field which can be received by the interface device */ 
 uint8_t    convention;   /* Convention of the communication */
 uint8_t    retries;      /* Number of retries */  
 uint8_t    rc_bytes;     /* Length of EDC */
 uint8_t    (* checksum)(uint8_t * data, uint32_t len, uint8_t *rc); /* Pointer to the function that computes EDC */
 uint8_t    more;  /* More data bit */
 uint8_t    previous_block[5]; /* To store the last R-block */
} t1_TypeDef;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void T1_Protocol_Init(t1_TypeDef * t1, uint32_t sc_freq);
int32_t T1_SetParameter(t1_TypeDef * t1, uint8_t param_type, uint32_t value);
int32_t T1_Negotiate_IFSD(t1_TypeDef * t1, uint8_t nad, uint8_t ifsd);
int32_t T1_APDU(t1_TypeDef * t1, uint8_t nad, void *apdu_c, uint32_t apdu_c_len, void *apdu_r, uint32_t apdu_r_len);

#endif /* __T1_PROTOCOL_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
