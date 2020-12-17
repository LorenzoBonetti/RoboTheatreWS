/************************** (C) COPYRIGHT 2010 STMicroelectronics ********************
* THE PRESENT FIRMWARE/SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
**************************************************************************************/

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __SERIAL_PROTOCOL__
#define __SERIAL_PROTOCOL__

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/* Exported constants --------------------------------------------------------*/

#define TMsg_EOF     0xF0
#define TMsg_BS      0xF1
#define TMsg_BS_EOF  0xF2
#define TMsg_MaxLen  1024

#define CMD_Default_WAIT            3000  // Ms

/* Exported types ------------------------------------------------------------*/

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;
typedef char s8;
typedef short s16;
typedef int s32;
typedef s32 Bool;

/**
  * \struct TMsg
  * \brief Data structure that contains messages that will be sent or received through the serial interface.
  **/
typedef struct {
  u32 Len;
  u8 Data[TMsg_MaxLen];
} TMsg;

/* Module private variables --------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

// Build a Number from an array (LSB first)
u32 Deserialize(u8 *Source, u32 Len);
int32_t Deserialize_s32(uint8_t *Source, uint32_t Len);
// Build an array from the u32 (LSB first)
void Serialize(u8 *Dest, u32 Source, u32 Len);
void SerializeSigned(u8 *Dest, s32 Source, u32 Len);
void Serialize_s32(uint8_t *Dest, int32_t Source, uint32_t Len);


int ByteStuffCopy(u8 *Dest, TMsg *Source);
int ReverseByteStuffCopy(TMsg *Dest, u8 *Source);
void CHK_ComputeAndAdd(TMsg *Msg);
int CHK_CheckAndRemove(TMsg *Msg);

#endif /* __SERIAL_PROTOCOL__ */

/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
