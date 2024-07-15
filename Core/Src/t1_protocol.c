/**
  ******************************************************************************
  * @file    Smartcard_T1/src/t1_protocol.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    13-May-2013
  * @brief   This file provides all the functions for the T=1 protocol.
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
#include "t1_protocol.h"
#include "checksum.h"
#include "t1_hal.h"
#include <string.h>


/* Private typedef -----------------------------------------------------------*/
/* State should be != DEAD after reset/init */
enum {
 SENDING,
 RECEIVING,
 RESYNCH,
 DEAD
};

/* Private define ------------------------------------------------------------*/
/* I block */
#define T1_I_SEQ_SHIFT     6    /* N(S) position (bit 7) */ 

/* R block */
#define T1_IS_ERROR(pcb)   ((pcb) & 0x0F)
#define T1_EDC_ERROR       0x01 /* [b6..b1] = 0-N(R)-0001 */
#define T1_OTHER_ERROR     0x02 /* [b6..b1] = 0-N(R)-0010 */
#define T1_R_SEQ_SHIFT     4    /* N(R) position (b5) */

/* S block  */
#define T1_S_RESPONSE  0x20   /* If response: set bit b6, if request reset b6 in PCB S-Block */
#define T1_S_RESYNC    0x00   /* RESYNCH: b6->b1: 000000 of PCB S-Block */
#define T1_S_IFS       0x01   /* IFS: b6->b1: 000001 of PCB S-Block */
#define T1_S_ABORT     0x02   /* ABORT: b6->b1: 000010 of PCB S-Block */
#define T1_S_WTX       0x03   /* WTX: b6->b1: 000011 of PCB S-Block */

#define NAD                 0  /* NAD byte positon in the block */
#define PCB                 1  /* PCB byte positon in the block */
#define LEN                 2  /* LEN byte positon in the block */
#define DATA                3  /* The positon of the first byte of INF field in the block */

/* Private macro -------------------------------------------------------------*/
#define T1_S_IS_RESPONSE(pcb) ((pcb) & T1_S_RESPONSE)
#define T1_S_TYPE(pcb)  ((pcb) & 0x0F)
#define SWAP_NIBBLES(x)         ((x >> 4) | ((x & 0xF) << 4))

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void T1_SetDefaults(t1_TypeDef * t1);
static void T1_SetChecksumType(t1_TypeDef * t1, uint8_t chsum_type);
static uint8_t T1_GetBlockType(uint8_t pcb);
static uint8_t T1_GetBlockSequence(uint8_t pcb);
static uint32_t T1_RebuildRBlock(t1_TypeDef *t1, uint8_t *block);
static uint32_t T1_ComputeChecksum(t1_TypeDef * t1, uint8_t *data, uint32_t len);
static uint32_t T1_VerifyChecksum(t1_TypeDef * t1, uint8_t *rbuf, uint32_t len);
static uint32_t T1_BuildBlock(t1_TypeDef * t1, uint8_t *TxBlock, uint8_t nad, uint8_t pcb, buffer_TypeDef *TxBufferStruct, uint32_t *TxInfFieldLength);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Set default T=1 protocol parameters.
  * @param  t1: pointer to t1 stucture of T=1 protocol.
  * @retval None.
  */
static void T1_SetDefaults(t1_TypeDef * t1)
{
  t1->retries = 3;
  t1->ifsc = 32; /* Maximum length of Information field which can be received by the card (by default set to 32)*/ 
  t1->ifsd = 32; /* Maximum length of Information field which can be received by the interface (by default set to 32) */
  t1->nr = 0;    /* N(R): the number of the expected l-block */
  t1->ns = 0;    /* N(S): the send-sequence number of the block */
  t1->wtx = 1;   /* WTX: waiting time extension */
}


/**
  * @brief  Set the checksum type to be used.
  * @param  t1: pointer to t1 stucture of T=1 protocol.
  * @param  chsum_type: the checksum type.
  *   This parameter can be one of the following values:
  *     @arg T1_PROTOCOL_CHECKSUM_LRC: select LRC.
  *     @arg T1_PROTOCOL_CHECKSUM_CRC: select CRC.
  * @retval None.
  */
static void T1_SetChecksumType(t1_TypeDef * t1, uint8_t chsum_type)
{
  switch (chsum_type)
  {
  case T1_PROTOCOL_CHECKSUM_LRC:  /* Select LRC calculation */
    t1->rc_bytes = 1;
    t1->checksum = csum_lrc_compute;
    break;
    
  case T1_PROTOCOL_CHECKSUM_CRC:   /* Select CRC calculation */
    t1->rc_bytes = 2;
    t1->checksum = csum_crc_compute;
    break;
    
  default:
    t1->rc_bytes = 1;
    t1->checksum = csum_lrc_compute;
    break;
  }
}

/**
  * @brief  Initialize T=1 protocol.
  * @param  t1: pointer to t1 stucture of T=1 protocol.
  * @param  sc_freq: smartcard clock frequency.  
  * @retval None.
  */
void T1_Protocol_Init(t1_TypeDef * t1, uint32_t sc_freq) 
{
  /* Set default T=1 protocol parameters */
  T1_SetDefaults(t1); 
  
  /* Set parameter: LRC is selected for EDC */
  T1_SetParameter(t1, T1_PROTOCOL_CHECKSUM_LRC, 0);
  
  /* Set parameter: Sending state: prepare for sending */
  T1_SetParameter(t1, T1_PROTOCOL_STATE, SENDING);
  
  /* Reset "more" parameter */
  T1_SetParameter(t1, T1_PROTOCOL_MORE, 0);
  
  /* Set the direct convention */
  T1_SetParameter(t1, T1_PROTOCOL_CONVENTION, DIRECT_CONV); 
  
  /* Set the etu value in us */
  T1_SetParameter(t1, T1_PROTOCOL_ETU, 372000000/sc_freq); 
  
  /* Set the smartcard frequency to be used by the protocol */
  T1_SetParameter(t1, T1_PROTOCOL_FREQUENCY, sc_freq); 
}


/**
  * @brief  Set T=1 protocol parameters.
  * @param  t1: pointer to t1 stucture of T=1 protocol.
  * @param  type: the parameter to be settled.
  *   This parameter can be one of the following values:
  *     @arg T1_PROTOCOL_CHECKSUM_LRC: select LRC.
  *     @arg T1_PROTOCOL_CHECKSUM_CRC: select CRC.
  *     @arg T1_PROTOCOL_IFSC: IFS for smart card.
  *     @arg T1_PROTOCOL_STATE: status of the T=1 communication.
  *     @arg T1_PROTOCOL_MORE: "more" bit.
  * @param  value: the new value of the parameter.
  * @retval 
  *     - 0: parameter is settled.
  *     - (-1): unsupported parameter.
  */
int32_t T1_SetParameter(t1_TypeDef * t1, uint8_t param_type, uint32_t value)
{
  switch (param_type)
  {
  case T1_PROTOCOL_CHECKSUM_LRC:  /* Set the EDC type */
  case T1_PROTOCOL_CHECKSUM_CRC:
    T1_SetChecksumType(t1, param_type);
    break;
    
  case T1_PROTOCOL_IFSC:  /* Set the IFSC value */
    t1->ifsc = (uint8_t)value;
    break;
    
  case T1_PROTOCOL_IFSD:  /* Set the IFSD value */
    t1->ifsd = (uint8_t)value;
    break;
    
  case T1_PROTOCOL_STATE: /* the card state */
    t1->state = (uint8_t)value;
    break;
    
  case T1_PROTOCOL_MORE: /* More bit */
    t1->more = (uint8_t)value;
    break;
    
  case  T1_PROTOCOL_CONVENTION: /* The convention type */
    t1->convention = (uint8_t)value;
    break;
    
  case  T1_PROTOCOL_ETU: /* The etu value in us */
    t1->etu_us = value;
    break;                
    
  case  T1_PROTOCOL_FREQUENCY: /* The frequency of the smart card in Hz */
    t1->frequency = value;
    break;                
    
  case T1_PROTOCOL_BWI:  /* BWI value */
    t1->bwi = value;
    break;  
    
  case T1_PROTOCOL_CWI:  /* CWI value */
    t1->cwi = value;
    break;            
    
  default:
    
    /* ---Unsupported parameter--- */      
    return -1;
  }
  
  return 0;
}


/**
  * @brief  Send APDU command and receive APDU response through T=1 protocol.
  * @param  t1: pointer to t1 stucture of T=1 protocol.
  * @param  nad: nad value (Node Address).  
  * @param  apdu_c: pointer to ADPU command buffer.
  * @param  apdu_c_len: the length of the command ADPU buffer.
  * @param  apdu_r: pointer to ADPU command buffer.
  * @param  apdu_r_len: pointer to ADPU command buffer.
  * @retval the status of the APDU transation. The returned value can
  *   be one of the following:
  *     - if >0 : the length of the APDU response when the APDU transaction
  *      has suceeded.
  *     - (-1): in case of communication error.
  */
int32_t T1_APDU(t1_TypeDef * t1, uint8_t nad, void *apdu_c, uint32_t apdu_c_len, void *apdu_r, uint32_t apdu_r_len)
{
  buffer_TypeDef sbuf, rbuf, tbuf;
  uint8_t sdata[T1_BUFFER_SIZE], sblk[5];
  uint32_t slen, retries, resyncs, sent_length = 0;
  size_t last_send = 0;
  uint8_t pcb;
  int32_t n;
  
  if (apdu_c_len == 0)
  {
    return -1;
  }
  
  /* we can't talk to a dead card / reader. Reset it! */
  if (t1->state == DEAD)
  {
    
    /* ---T=1 state machine is DEAD. The card should be initialized--- */   
    return -1;
  }
  
  t1->state = SENDING;
  retries = t1->retries;
  resyncs = 3;
  
  /* Initialize the send buffer */
  Buffer_set(&sbuf, (void *)apdu_c, apdu_c_len);
  
  /* Initialize the receive buffer */
  Buffer_init(&rbuf, apdu_r, apdu_r_len);
  
  /* ---Sending I-Block--- */
  
  /* Build the first I-block (sdata) and return the lenght of the buffer to send */
  slen = T1_BuildBlock(t1, sdata, nad, T1_I_BLOCK, &sbuf, (uint32_t *)&last_send);
  
  while (1)
  {
    retries--;
    
    /* Send the block (sdata) and get the response in the same buffer */
    n = T1_TxRxBlock(t1, sdata, slen);
    
    if (n < 0) /* Receive Parity error or BWT timeout or block malformed */
    {
      
      /* ---Response Error Occured--- */         
      /* ISO 7816-3 Rule 7.4.2 */
      if (retries == 0)
      {
        /* Resynchronization required */ 
        goto resync;
      }
      
      /* ISO 7816-3 Rule 7.2: retransmit R-Block */
      if (T1_R_BLOCK == T1_GetBlockType(t1->previous_block[PCB]))
      {
        
        slen = T1_RebuildRBlock(t1, sdata);
        
        continue;
      }
      
      if (n == -2) /* Parity Error */
      {
        
        /* ISO 7816-3 Rule 7.1  */
        slen = T1_BuildBlock(t1, sdata, nad, T1_R_BLOCK | T1_EDC_ERROR,NULL, NULL);
      }
      else if((n == -1) || (n == -3)) /* BWT timeout or block malformed */
      {
        /* ---BWT timeout or block malformed, Sending R-Block--- */
        /* ISO 7816-3 Rule 7.1 */
        slen = T1_BuildBlock(t1, sdata, nad, T1_R_BLOCK | T1_OTHER_ERROR, NULL, NULL);
      }
      continue;
    }
    
    
    /* Wrong NAD or illegal LEN == 0xFF  */
    if ((sdata[NAD] != SWAP_NIBBLES(nad)) || (sdata[LEN] == 0xFF)) 
    {
      /* ---Wrong NAD or Block Length...--- */
      /* ISO 7816-3 Rule 7.4.2 */
      if (retries == 0)
      {
        /* ---Resynchronization required--- */
        goto resync;
      }
      
      /* ISO 7816-3 Rule 7.2: If the previous sent block was R-Block, then retransmit it */
      if (T1_R_BLOCK == T1_GetBlockType(t1->previous_block[PCB]))
      {                              
        slen = T1_RebuildRBlock(t1, sdata);
        continue;
      }
      
      /* Tranmsit R-Block signaling Other error */
      slen = T1_BuildBlock(t1, sdata, nad, T1_R_BLOCK | T1_OTHER_ERROR,NULL, NULL);
      continue;
    }
    /* Wrong CRC */
    if (!T1_VerifyChecksum(t1, sdata, n))  
    {
      /* ---Checksum failed--- */
      /* ISO 7816-3 Rule 7.4.2 */
      if (retries == 0)
      {
        goto resync;
      }
      
      /* ISO 7816-3 Rule 7.2: If the previous sent block was R-Block, then retransmit it  */
      if (T1_R_BLOCK == T1_GetBlockType(t1->previous_block[PCB]))
      {                          
        slen = T1_RebuildRBlock(t1, sdata);
        
        continue;
      }
      
      /* If the received block is not R-Block, then transmit a R-Block signaling EDC error  */
      slen = T1_BuildBlock(t1, sdata, nad, T1_R_BLOCK | T1_EDC_ERROR, NULL, NULL);
      continue;
    }
    
    /* Get the pcb value of the received block */
    pcb = sdata[PCB];
    
    /* Determine the type of the received block */
    switch (T1_GetBlockType(pcb))
    {
    case T1_R_BLOCK:
      /* ---R-Block--- */
      /* length != 0x00 (illegal) or b6 of pcb is set*/
      if ((sdata[LEN] != 0x00) || (pcb & 0x20))  
      {
        /* ---Wrong R-Block received--- */
        /* ISO 7816-3 Rule 7.4.2 */
        if (retries == 0)
        {
          goto resync;
        }
        
        /* ISO 7816-3 Rule 7.2 (if the previous block send was R-Block then retransmit it  */
        if (T1_R_BLOCK == T1_GetBlockType(t1->previous_block[PCB]))
        {                              
          slen = T1_RebuildRBlock(t1, sdata);
          continue;
        }
        
        
        /* If the previous block sent wasn't R-Block then transmit R-Block signaling "Other Error"  */
        slen = T1_BuildBlock(t1, sdata, nad, T1_R_BLOCK | T1_OTHER_ERROR, NULL, NULL);
        continue;
      }
      /* Wrong sequence number: we have received N(R) != N(S) when we are not in chaining process (more = 0).
      Normally when we sent an I-Block and an R-Bloks is received and we are not in chaining mode, N(R) should
      be equal to N(S) (refer to rule 7.1) */
      if (((T1_GetBlockSequence(pcb) != t1->ns) && (! t1->more))) 
      {
        /* ---Wrong sequence number received: Sequence number received: T1_GetBlockSequence(pcb), expected: t1->ns, more: t1->more--- */
        /* ISO 7816-3 Rule 7.2: if the previous block send was R-Block then retransmit it  */
        if (T1_R_BLOCK == T1_GetBlockType(t1->previous_block[PCB]))
        {
          /* ---Rule 7.2: retransmit R-Block--- */
          slen = T1_RebuildRBlock(t1, sdata);
          continue;
        }
        
        /* ISO 7816-3 Rule 7.4.2 */
        if (retries == 0)
        {
          goto resync;
        }
        
        /* ---Transmit R-Block signaling Other error--- */
        /* If the previous block sent wasn't R-Block then transmit R-Block signaling "Other Error"  */     
        slen = T1_BuildBlock(t1, sdata, nad, T1_R_BLOCK | T1_OTHER_ERROR, NULL, NULL);
        continue;
      }
      
      if (t1->state == RECEIVING)
      {
        /* ISO 7816-3 Rule 7.2: if the previous block send was R-Block then retransmit it  */
        if (T1_R_BLOCK == T1_GetBlockType(t1->previous_block[PCB]))
        {
          /* ---Rule 7.2: retransmit R-Block--- */
          slen = T1_RebuildRBlock(t1, sdata);
          continue;
        }
        
        /* ---Sending R-Block--- */
        slen = T1_BuildBlock(t1, sdata, nad, T1_R_BLOCK, NULL, NULL);
        break;
      }
      
      /* Rule 2.2 (case of chaining): If the card requests the next
      * I-block sequence number, this means that it has received 
      * the previous block successfully */
      if (T1_GetBlockSequence(pcb) != t1->ns)
      {
        Buffer_get(&sbuf, NULL, last_send);
        sent_length += last_send; /* store the data lenght cumulation */
        last_send = 0;
        t1->ns ^= 1; /* Inverse the sequence number */
      }
      
      /* If there's no data available, the ICC
      * shouldn't be asking for more */
      if (Buffer_avail(&sbuf) == 0)
      {
        goto resync;
      }
      
      /* Send an I-Block */
      slen = T1_BuildBlock(t1, sdata, nad, T1_I_BLOCK, &sbuf, (uint32_t *)&last_send);
      break;
      
    case T1_I_BLOCK:
      
      /* The first I-block sent by the ICC indicates
      * the last block we sent was received successfully */
      if (t1->state == SENDING)
      {
        Buffer_get(&sbuf, NULL, last_send);
        last_send = 0;
        t1->ns ^= 1;
      }
      
      /* Switch to receive mode */
      t1->state = RECEIVING;
      
      /* If the block sent by the card doesn't match
      * what we expected it to send, reply with
      * an R-block */
      if (T1_GetBlockSequence(pcb) != t1->nr)
      {
        /* ---wrong N(R), Transmit R-Block signaling Other error--- */
        slen = T1_BuildBlock(t1, sdata, nad, T1_R_BLOCK | T1_OTHER_ERROR, NULL, NULL);
        continue;
      }
      /* Inverse N(R) value */
      t1->nr ^= 1;
      
      if (Buffer_put(&rbuf, sdata + 3, sdata[LEN]) < 0)
      {
        /* ---Buffer overrun by (sdata[LEN] - (rbuf.size - rbuf.tail)) bytes--- */
        goto error;
      }
      /* If more bit (b6 of I-Block) is equal to 0 the APDU is completed  */
      if ((pcb & T1_MORE_BLOCKS) == 0) 
      {
        goto done;
      }
      
      /* Send R-Block for chaining  */
      slen = T1_BuildBlock(t1, sdata, nad, T1_R_BLOCK, NULL, NULL);
      break;
      
      
    case T1_S_BLOCK:
      
      /* If the S-Block received is a response while the reader is
      in resynchronization state after it has sent a synchronization request */
      if (T1_S_IS_RESPONSE(pcb) && (t1->state == RESYNCH) && (T1_S_TYPE(pcb) == T1_S_RESYNC))
      {
        /* ---S-Block answer received for resynchronization--- */
        /* ISO 7816-3 Rule 6.3 */
        t1->state = SENDING;
        sent_length = 0;
        last_send = 0;
        resyncs = 3;
        retries = t1->retries;
        Buffer_init(&rbuf, apdu_r, apdu_r_len);
        
        /* Send I-Block  */
        slen = T1_BuildBlock(t1, sdata, nad, T1_I_BLOCK, &sbuf, (uint32_t *)&last_send);
        continue;
      }
      
      /* There are 5 types of S-Block responses: RESYNCH, IFS, ABORT, WTX, VPP state error:
      - RESYNCH: is treated in the condition above.
      - IFS response is not allowed here because it's managed in T1_Negotiate_IFSD function, so the card is not allowed to send IFS response. 
      - ABORT response is not allowed since we don't send in any case an ABORT request.
      - WTX response is allowed only from interface device side, so the card is not allowed to send this type of response.
      - VPP state error: we suppose that it was an error, we send en R-block 
      --> only RESYNCH response is allowed. All other responses are treated as errors */
      if (T1_S_IS_RESPONSE(pcb))  
      {
        /* ISO 7816-3 Rule 7.4.2 */
        if (retries == 0)
        {
          goto resync; 
        }
        
        /* ISO 7816-3 Rule 7.2 */
        if (T1_R_BLOCK == T1_GetBlockType(t1->previous_block[PCB]))
        {                                    
          slen = T1_RebuildRBlock(t1, sdata);
          continue;
        }
        
        /* Transmit R-Block signaling "Other Error"  */       
        slen = T1_BuildBlock(t1, sdata,nad, T1_R_BLOCK | T1_OTHER_ERROR, NULL, NULL);
        continue;
      }
      
      Buffer_init(&tbuf, sblk, sizeof(sblk));
      
      /* ---S-Block request received--- */
      
      switch (T1_S_TYPE(pcb))
      {
      case T1_S_RESYNC:  /* The SmartCard sent resynchronization request */
        
        /* If length different from 0 (see ISO7816 - 9.4.3) */
        if (sdata[LEN] != 0)
        {
          /* ---Wrong length: sdata[LEN]--- */
          /* ---Sending R-Block signaling other error--- */
          slen = T1_BuildBlock(t1, sdata, nad, T1_R_BLOCK | T1_OTHER_ERROR, NULL, NULL);
          continue;
        }
        
        /* The card is not allowed to send a resync request. Process a resychronization request */
        goto resync;
        
      case T1_S_ABORT:  /* The SmartCard sent a data abortion request */
        /* If length different from 0 (see ISO7816 - 9.4.3) */
        if (sdata[LEN] != 0)
        {
          /* ---Wrong length: sdata[LEN]--- */
          /* ---Sending R-Block signaling other error--- */
          
          /* See Annex A: A.3.3 scenario 16 */
          slen = T1_BuildBlock(t1, sdata, nad, T1_R_BLOCK | T1_OTHER_ERROR, NULL, NULL);
          continue;
        }
        
        /* ISO 7816-3 Rule 9 */
        /* ---Abort requested, sending S-Abort response--- */
        break;
        
      case T1_S_IFS:  /* The SmartCard sent an IFS request */
        if (sdata[LEN] != 1)
        {
          /* ---Wrong length: sdata[LEN]--- */
          /* ---Sending R-Block signaling other error--- */
          
          /* See Annex A: A.3.3 scenario 16 */
          slen = T1_BuildBlock(t1, sdata, nad, T1_R_BLOCK | T1_OTHER_ERROR, NULL, NULL);
          continue;
        }
        /* ---The card sent an S(IFS_request) with ifs = sdata[DATA]--- */
        if ((sdata[DATA] == 0) || (sdata[DATA] == 0xFF)) /* IFSC should be: 0x01 to 0xFE */
        {
          goto resync;
        }
        /* Store the new IFSC value  */
        t1->ifsc = sdata[DATA];
        Buffer_putc(&tbuf, sdata[DATA]);
        break;
        
      case T1_S_WTX: /* The SmartCard sent a WTX request */
        if (sdata[LEN] != 1)
        {
          /* ---Wrong length: sdata[LEN]--- */                                
          /* Send an R-Block signaling "Other Error" */       
          slen = T1_BuildBlock(t1, sdata, nad, T1_R_BLOCK | T1_OTHER_ERROR, NULL, NULL);
          continue;
        }
        /* ---The card sent S(WTX_request) with wtx=sdata[DATA]--- */
        t1->wtx = sdata[DATA];
        Buffer_putc(&tbuf, sdata[DATA]);
        break;
        
      default:
        
        /* Wrong S-Block  */                                
        goto resync;
      } /* switch (T1_S_TYPE(pcb)) */
      
      /* Send the S-block response according to the received S-Block request */
      slen = T1_BuildBlock(t1, sdata, nad, T1_S_BLOCK | T1_S_RESPONSE | T1_S_TYPE(pcb),&tbuf, NULL);
    } /* End switch (T1_GetBlockType(pcb))*/
    
    /* Everything went splendid, initialize the number of retries */
    retries = t1->retries;
    continue;
    
  resync:
    /* the number or resyncs is limited, too */
    /* ISO 7816-3 Rule 6.4 */
    if (resyncs == 0)
    {
      goto error;
    }
    
    /* ---Sending S-Block RESYNCH Req--- */
    /* ISO 7816-3 Rule 6 */
    resyncs--;
    t1->ns = 0;
    t1->nr = 0;
    slen = T1_BuildBlock(t1, sdata, nad, T1_S_BLOCK | T1_S_RESYNC, NULL, NULL);
    t1->state = RESYNCH;
    t1->more = 0;
    retries = 1;
    continue;
  }
  
done:
  return Buffer_avail(&rbuf);
  
error:
  /* ---SCard should be initialized or deactivated--- */
  t1->state = DEAD;  /* At this stage, the interface device should reset the card */
  return -1;
}


/**
  * @brief  Determine the type of a block.
  * @param  pcb: the PCB field of the block (the 2nd byte of the prologue field).
  * @retval the type of the block. The returned value can be one of 
  *   the following:
  *     - T1_I_BLOCK : in case of I-Block.
  *     - T1_R_BLOCK : in case of R-Block.
  *     - T1_S_BLOCK : in case of S-Block.
  */
static uint8_t T1_GetBlockType(uint8_t pcb)
{
  switch (pcb & 0xC0)
  {
  case T1_R_BLOCK:  return T1_R_BLOCK;
  
  case T1_S_BLOCK:  return T1_S_BLOCK;
  
  default: return T1_I_BLOCK;
  }
}

/**
  * @brief  Determine the sequence number of the block.
  * @param  pcb: the PCB field of the block (the 2nd byte of the prologue field).
  * @retval the sequence number of the block. The returned value can
  *   be one of the following:
  *     - 1 : sequence number equal to 1.
  *     - 0 : sequence number equal to 0.
  */
static uint8_t T1_GetBlockSequence(uint8_t pcb)
{
  switch (pcb & 0xC0)
  {
  case T1_R_BLOCK:  return (pcb >> T1_R_SEQ_SHIFT) & 1; /* Get bit5 of the PCB's R-block */
  
  case T1_S_BLOCK:  return 0; /* Does not have sequence number */
  
  default:  return (pcb >> T1_I_SEQ_SHIFT) & 1;  /* Get bit7 of the PCB's I-block */
  }
}

/**
  * @brief  Build a Block.
  * @param  t1: pointer to t1 stucture of T=1 protocol.
  * @param  TxBlock: a pointer to the buffer of the block to be sent.  
  * @param  nad: the value of the node address.
  * @param  pcb: the PCB byte to be sent in the prologue field.
  * @param  TxBufferStruct: a pointer to the structure of the buffer to be sent.
  * @param  TxInfFieldLength: a pointer to a variable that contains the lenght of
  *         the information field (INF).
  * @retval the lenght of the whole block (NAD+PCB+LEN+INF+EDC).
  */
static uint32_t T1_BuildBlock(t1_TypeDef * t1, uint8_t *TxBlock, uint8_t nad, uint8_t pcb, buffer_TypeDef *TxBufferStruct, uint32_t *TxInfFieldLength)
{
  uint32_t len;
  uint8_t more = 0;
  
  /* if TxBufferStruct=0: len <- 0,
  if TxBufferStruct!=0: len <- value returned by Buffer_avail(TxBufferStruct) */
  len = TxBufferStruct ? Buffer_avail(TxBufferStruct) : 0;
  
  if (len > t1->ifsc) /* If the information field length is greater than */
  {                   /* the maximum information field of the card: chaining function */
    pcb |= T1_MORE_BLOCKS;
    len = t1->ifsc;
    more = 1;
  }
  
  /* Add the sequence number */
  switch (T1_GetBlockType(pcb))
  {
  case T1_R_BLOCK:
    pcb |= t1->nr << T1_R_SEQ_SHIFT;
    break;
    
  case T1_I_BLOCK:
    pcb |= t1->ns << T1_I_SEQ_SHIFT;
    t1->more = more;                 
    break;
  }
  
  /* Build the Prologue field */
  TxBlock[0] = nad; /* NAD field */
  TxBlock[1] = pcb; /* PCB field */
  TxBlock[2] = (uint8_t)len; /* The lenght of the data field (INF field) */
  
  if (len)
  {
    memcpy(TxBlock + 3, Buffer_head(TxBufferStruct), len);
  }
  
  if (TxInfFieldLength)
  {
    *TxInfFieldLength = len; /* At this stage len contains the length of the data field (INF field) */
  }
  
  /* Compute the checksum of the buffer (from NAD to INF) and
  return the value of the lenght of the whole buffer NAD -> CRC */
  len = T1_ComputeChecksum(t1, TxBlock, len + 3);
  
  /* memorize the last sent block */
  /* only 4 bytes since we are only interesed in R-blocks */
  memcpy(t1->previous_block, TxBlock, 3 + t1->rc_bytes);
  
  /* Return the lenght of the whole buffer (NAD+PCB+LEN+INF+EDC) */
  return len;
}


/**
  * @brief  rebuild the last sent R-Block.
  * @param  t1: pointer to t1 stucture of T=1 protocol.
  * @param  block: a pointer to a buffer to send.  
  * @retval the length of the block. The returned value can
  *   be one of the following:
  *     - =!0 : the length of the R-Block.
  *     - 0 : if the previous block wasn't a R-Block.
  */
static uint32_t T1_RebuildRBlock(t1_TypeDef *t1, uint8_t *block)
{
  uint8_t pcb = t1 -> previous_block[1];
  
  /* Copy the last sent block */
  if (T1_R_BLOCK == T1_GetBlockType(pcb))
  {
    memcpy(block, t1->previous_block, 3 + t1->rc_bytes);
  }
  else
  {
    /* ---previous block was not R-Block: PCB = pcb--- */
    return 0;
  }
  
  /* 3 bytes of prologue field + EDC length */
  return (3 + t1->rc_bytes); 
}

/**
  * @brief  Compute the checksum value depnding on the used EDC (LRC or CRC).
  * @param  t1: a pointer to t1 stucture of T=1 protocol.
  * @param  data: a pointer to the data buffer to apply the checksum.  
  * @param  len: the length of the data buffer.
  * @retval the sum of the length of the buffer and the length of the Checksum fields.
  */
static uint32_t T1_ComputeChecksum(t1_TypeDef * t1, uint8_t *data, uint32_t len)
{       
  /* Compute the checksum of data and put it in (data+len) position*/
  return len + t1->checksum(data, len, data + len);
}


/**
  * @brief  verify the checksum.
  * @param  t1: a pointer to t1 stucture of T=1 protocol.
  * @param  rbuf: a pointer to the buffer to check its checksum.  
  * @param  len: the length of the buffer including the EDC length.
  * @retval the status of the checksum. The returned value can
  *   be one of the following:  
  *     - 1: checksum ok.
  *     - 0: checksum error.
  */
static uint32_t T1_VerifyChecksum(t1_TypeDef * t1, uint8_t *rbuf, uint32_t len)
{
  uint8_t csum[2];
  int32_t m, n;
  
  /* Compute the length of the block with EDC not included (NAD->INF) */
  m = len - t1->rc_bytes;  
  
  /* Store the length of EDC */
  n = t1->rc_bytes;
  
  if (m < 0)
  {
    return 0; /* Checksum Error */
  }
  
  /* Compute the checksum of the buffer pointed by rbuf and put it in csum buffer: LRC or CRC */
  t1->checksum(rbuf, m, csum);
  
  /* Compare the received Checksum (EDC) to the expected one */
  if (!memcmp(rbuf + m, csum, n))
  {
    return 1; /* Checksum OK */
  }
  
  return 0; /* Checksum Error */
}

/**
  * @brief  Send IFS request to indicate a new IFSD that can support (reader side).
  * @param  t1: a pointer to t1 stucture of T=1 protocol.
  * @param  nad: the value of the node address.
  * @param  ifsd: the new value of the reader IFS to be negociated.
  * @retval the status of the IFSD request communication. The returned value can
  *   be one of the following: 
  *     - (>0) if the transmisson was succeded.
  *     - (-1): if the transmission was failed.
  */
int32_t T1_Negotiate_IFSD(t1_TypeDef * t1, uint8_t nad, uint8_t ifsd)
{
  buffer_TypeDef sbuf;
  uint8_t sdata[5];
  
  uint32_t slen;
  int32_t retransmission_attempt;
  int32_t resychronization_attempt = 3; /* Number of resynchronisation attempt */
  size_t snd_len;
  int32_t n;
  uint8_t snd_buf[1];  /* Information field of S-Block to be sent */
  
  /* Get the number of retries */
  retransmission_attempt = t1->retries;
  
  /* S-block IFSD request */
  snd_buf[0] = ifsd;
  snd_len = 1;
  
  /* Initialize send/recv buffer */
  Buffer_set(&sbuf, (void *)snd_buf, snd_len);
  
  /* ---Sending IFSD request with value ifsd--- */
  
  while (1)
  {
    /* Build the S-block with nad=0000 (0), PCB=11000001 (0xC0 | 0x01) and return the S-Block length */
    slen = T1_BuildBlock(t1, sdata, nad, T1_S_BLOCK | T1_S_IFS, &sbuf, NULL);
    
    /* Send and the S-block IFS request and get the S-block IFS response */
    n = T1_TxRxBlock(t1, sdata, slen);
    
    retransmission_attempt--;
    
    if ((n < 0)                       /* Parity error or block malformed or BWT timeout */
        || (sdata[DATA] != ifsd)  /* Wrong ifsd received */
        || (sdata[NAD] != SWAP_NIBBLES(nad)) /* wrong NAD */
        || (!T1_VerifyChecksum(t1, sdata, n)) /* checksum failed */
        || (n != 4 + t1->rc_bytes)       /* wrong frame length */
        || (sdata[LEN] != 1)   /* wrong data length */
        || (sdata[PCB] != (T1_S_BLOCK | T1_S_RESPONSE | T1_S_IFS))) /* wrong PCB */
    {
      /* ISO 7816-3 Rule 7.4.2 */
      if (retransmission_attempt == 0)
      {
        goto resynchronization;
      }                    
      continue;
    }
    else
    {
      /* ---IFSD request suceeded--- */
      return n; 
    }
  }
  
resynchronization: /* Resynchronization */
  
  do /* Do maximum three resynchronisation attempts */
  { 
    /* the number or resyncs is limited, too */
    /* ISO 7816-3 Rule 6.4 */
    resychronization_attempt--;                    
    
    if (resychronization_attempt == 0)
    {
      /* ---SCard should be initialized or deactivated--- */
      t1->state = DEAD;
      return -1;                        
    }
    
    /* ---Sending S-Block RESYNCH Req n resychronization_attempt--- */
    /* ISO 7816-3 Rule 6 */
    t1->ns = 0;
    t1->nr = 0;
    slen = T1_BuildBlock(t1, sdata, nad, T1_S_BLOCK | T1_S_RESYNC, NULL, NULL);
    t1->state = RESYNCH;
    t1->more = 0;
    /* Send the S-block (resychronization request */
    n = T1_TxRxBlock(t1, sdata, slen);
    
  }
  while((n <0) 
        || (sdata[NAD] != SWAP_NIBBLES(nad)) /* wrong NAD */
        || (!T1_VerifyChecksum(t1, sdata, n)) /* checksum failed */
        || (n != 3 + t1->rc_bytes)       /* wrong frame length */
        || (sdata[LEN] != 0)   /* wrong data length */
        || (sdata[PCB] != (T1_S_BLOCK | T1_S_RESPONSE | T1_S_RESYNC))); /* wrong PCB */
  
  
  /* If the resynchronization has succeeded then make one S-IFS request */
  /* Build the S-block with nad=0000 (0), PCB=11000001 (0xC0 | 0x01) and return the S-Block length */
  slen = T1_BuildBlock(t1, sdata, nad, T1_S_BLOCK | T1_S_IFS, &sbuf, NULL);
  
  /* Send and the S-block IFS request and get the S-block IFS response */
  n = T1_TxRxBlock(t1, sdata, slen);
  
  /* If the S-block IFS response has failed, deactivate the card */
  if ((n == -2) || (n == -3)       /* Parity error or block malformed */
      || (sdata[DATA] != ifsd)          /* Wrong ifsd received */
      || (sdata[NAD] != SWAP_NIBBLES(nad)) /* wrong NAD */
      || (!T1_VerifyChecksum(t1, sdata, n)) /* checksum failed */
      || (n != 4 + t1->rc_bytes)               /* wrong frame length */
      || (sdata[LEN] != 1)   /* wrong data length */
      || (sdata[PCB] != (T1_S_BLOCK | T1_S_RESPONSE | T1_S_IFS))) /* wrong PCB */
  {
    t1->state = DEAD;
    return -1;
  }
  else
  {
    /* ---IFSD request suceeded after resynchronization--- */
    return n;
  }
}

/**
  * @}
  */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
