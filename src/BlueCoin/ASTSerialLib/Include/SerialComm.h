/************************** (C) COPYRIGHT 2010 STMicroelectronics ********************
* THE PRESENT FIRMWARE/SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
**************************************************************************************/
#define CCOMPORT_NO_QT
#ifndef SERIALCOMM_H_
#define SERIALCOMM_H_

#ifdef CCOMPORT_NO_QT
    #include "CComPort_noQt.h"
#else
    #include "CComPort.h"
#endif
#include <stdint.h>

extern "C" {
#include "serial_protocol.h"
#include "Serial_CMD.h"
}



class SerialComm : public CComPort
{
protected:
	int FDevAddr;

public:
	SerialComm(void);
	~SerialComm();

public:
	int32_t SetDevAddr(int32_t Value);
	int32_t GetDevAddr(void);
	int32_t SetEndOfFrame_AT();
	int32_t SetEndOfFrame_ASTprotocol();
	int32_t ReceivedMSG(TMsg *Msg, double MaxWait);
	int32_t SendMsg(TMsg *Msg);
    int32_t SendSerialCmd(uint8_t Addr, uint8_t Cmd, uint32_t DataLen =0, uint8_t *Data=0);
    int32_t ReceiveSerialCmdReply(uint8_t Addr, uint8_t Cmd, u16 MSTimeout, uint32_t MaxLen=0, uint8_t *Data = 0);
    int32_t ReceiveSerialCmdAsync(uint8_t Addr, uint8_t *CmdReceived, u16 MSTimeout, uint32_t MaxLen, uint8_t *Data);
    int32_t DeleteAllReceivedMessages(uint8_t Addr, uint8_t Cmd, u16 MSTimeout, uint32_t MaxLen, uint8_t *Data);
	int32_t SendBuffer(unsigned char *My_Buffer,int32_t len);
	int32_t SendString(char *String);
	int32_t ReceiveString(char *String, int32_t MaxLen, double MaxWait);

};

#endif /* SERIALCOMM_H_ */
