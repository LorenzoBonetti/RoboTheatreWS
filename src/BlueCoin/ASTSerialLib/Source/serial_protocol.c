/************************** (C) COPYRIGHT 2010 STMicroelectronics ********************/
/**
* \file serial_protocol.c
*
* \brief This file provides some C useful function for the RVS serial protocol implementation.
*
* \author Central Labs
* \date 09/07/2010
* \version Version 2.0.1
*
* \section STLic ST Software License
* THE PRESENT FIRMWARE/SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*/
#include "../Include/serial_protocol.h"

/**
 * Build a Number from an array (LSB first).
 *
 * \param Source Source array (serialized number).
 * \param Len Length of the array.
 * \return Deserialized number.
**/
u32 Deserialize(u8 *Source, u32 Len)
// Build a Number from an array (LSB first)
{
  u32 app;
  app=Source[--Len];
  while(Len>0) {
    app<<=8;
    app+=Source[--Len];
  }
  return app;
}

/**
 * Build an array from the u32 (LSB first).
 *
 * \param Dest Destination array (serialized number).
 * \param Len Length of the array.
 * \param Source Deserialized number.
**/
void Serialize(u8 *Dest, u32 Source, u32 Len)
// Build an array from the u32 (LSB first)
{
  u32 i;
  for (i=0; i<Len; i++) {
    Dest[i] = Source & 0xFF;
    Source>>=8;
  }
}

void Serialize_s32(uint8_t *Dest, int32_t Source, uint32_t Len)
// Build an array from the uint32_t (LSB first)
{
  u32 i;
  for (i=0; i<Len; i++) {
    Dest[i] = Source & 0xFF;
    Source>>=8;
  } 
}

int32_t Deserialize_s32(uint8_t *Source, uint32_t Len)
// Build a Number from an array (LSB first)
{
  int32_t app;
  app=Source[--Len];
  while(Len>0) {
    app<<=8;
    app+=Source[--Len];
  }
  return app;
}

/**
 * Build an array from the u32 (LSB first).
 *
 * \param Dest Destination array (serialized number).
 * \param Len Length of the array.
 * \param Source Deserialized number.
**/
void SerializeSigned(u8 *Dest, s32 Source, u32 Len)
// Build an array from the u32 (LSB first)
{
  u32 i;
  for (i=0; i<Len; i++) {
    Dest[i] = Source & 0xFF;
    Source>>=8;
  }
}

/**
 * Create a message with escape special charactes from a generic one.
 *
 * \param Source Message without escape sequences.
 * \param Dest Destination array (message with escape sequences).
**/
int ByteStuffCopy(u8 *Dest, TMsg *Source)
{
  u32 i, Count;
  
  Count=0;
  for (i=0; i<Source->Len; i++) {
    switch(Source->Data[i]) {
      case TMsg_EOF:
        Dest[Count]=TMsg_BS;
        Count++;
        Dest[Count]=TMsg_BS_EOF;
        Count++;
        break;
      case TMsg_BS:
        Dest[Count]=TMsg_BS;
        Count++;
        Dest[Count]=TMsg_BS;
        Count++;
        break;
      default:
        Dest[Count]=Source->Data[i];
        Count++;
    }
  }
  Dest[Count]=TMsg_EOF;
  Count++;
  return Count;
}

/**
 * Create a message with escape special charactes from a generic one.
 *
 * \param Source Message without escape sequences.
 * \param Dest Destination array (message with escape sequences).
**/
int ReverseByteStuffCopy(TMsg *Dest, u8 *Source)
{
  int Count=0, State=0;

  while ((*Source)!=TMsg_EOF) {
    if (State==0) {
      if ((*Source)==TMsg_BS) {
        State=1;
      } else {
        Dest->Data[Count]=*Source;
        Count++;
      }
    } else {
      if ((*Source)==TMsg_BS) {
        Dest->Data[Count]=TMsg_BS;
        Count++;
      } else {
        if ((*Source)==TMsg_BS_EOF) {
          Dest->Data[Count]=TMsg_EOF;
          Count++;
        } else {
          return 0; // invalid sequence
        }
      }
      State=0;
    }
    Source++;
  }
  if (State!=0)
      return 0;
  Dest->Len=Count;
  return 1;
}

/**
 * Compute and add checksum to the message.
 *
 * \param Msg Message to be modified.
 **/
void CHK_ComputeAndAdd(TMsg *Msg)
{
  u8 CHK=0;
  u32 i;
  
  for(i=0; i<Msg->Len; i++) {
    CHK-=Msg->Data[i];
  }
  Msg->Data[i]=CHK;
  Msg->Len++;
}

/**
 * Compute and remove the checksum to the message.
 *
 * \param Msg Message to be checked and modified.
 **/
int CHK_CheckAndRemove(TMsg *Msg)
{
  u8 CHK=0;
  u32 i;
  
  for(i=0; i<Msg->Len; i++) {
    CHK+=Msg->Data[i];
  }
  Msg->Len--;
  return (CHK==0);
}

/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
