/************************** (C) COPYRIGHT 2010 STMicroelectronics ********************/
/**
* \file ASTSerialLib.cpp
* \class ASTSerialLib
*
* \brief This class is a
*
* ASTSerialLib class provides a set of high level methods that allow to easily communicate
* with the device through a serial interface.
*
* \author  Central Labs
* \date 03/02/2011
* \version Version 2.2.0
*
* \section STLic ST Software License
* THE PRESENT FIRMWARE/SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* \mainpage ASTSerialLib
* \section intro_sec Introduction
* The ASTSerialLib provides an easy to use API that allows software developers to easily write
* custom c++ proograms that can easily interact with the device through a Serial interface.
*/

#include "../Include/ASTSerialLib.h"
#include "string.h"
#include "stdio.h"
#include <iostream>

ASTSerialLib::ASTSerialLib()
{
    SC = new SerialComm();

    LinkActive = false;
}

void ASTSerialLib::SetCom(SerialComm *SC_external)
{
    SC = SC_external;
}

SerialComm * ASTSerialLib::GetCom()
{
    return SC;
}

ASTSerialLib::~ASTSerialLib()
{
    delete SC;
}

/**
 * .
 *
**/
void ASTSerialLib::SetEndOfFrame_AT()
{
    SC->SetEndOfFrame_AT();
}

/**
 * .
 *
**/
void ASTSerialLib::SetEndOfFrame_ASTprotocol()
{
    SC->SetEndOfFrame_ASTprotocol();
}

/**
 * Sets the COM port number on which the RVS is connected.
 *
 * \param PortNumber COM port number
**/
void ASTSerialLib::SetCOMPortNumber(int PortNumber)
{
    SC->SetPortNumber(PortNumber);
}

/**
 * Sets the COM port number on which the RVS is connected.
 *
 * \param PortSpeed COM Port speed
 * \return Returns -1 if an error occurs, 0 instead.
**/
int ASTSerialLib::Open(int PortSpeed)
{
    if(SC->Open(PortSpeed)==0) {
        return -1;
    } else {
        LinkActive = true;
        return 0;
    }
}

/**
 * Check if the COM port is a RS232
 *
 * \return Returns -1 if an error occurs, 1 if the COM Port type is RS232 otherwise 0.
**/
#ifdef _WIN32
int ASTSerialLib::COM_IsRS232()
{
    return SC->IsRS232();
}
#endif

/**
 * Closes the previously opened COM Port.
 *
 * \return Returns -1 if an error occurs, 0 instead.
**/
int ASTSerialLib::Close()
{
    LinkActive = false;
    return SC->Close()==0?-1:0;
}

/**
 * Resets the Module.
 *
 * \param DestDevAddr Device address on the serial bus.
 * \return Returns 1 if successful or -1 if an error occurs.
**/
int ASTSerialLib::ASTCmd_Reset(unsigned char DestDevAddr)
{
    return SC->SendSerialCmd(DestDevAddr, CMD_Reset);
}

/**
 * Sets the client address that will be used as source address
 * when sending commands.(The Identification of the host or G2 Robot)
 *
 * \param ClientAddr Address of the client in the serial bus
 * \return Returns -1 if an error occurs, 0 instead.
**/
int ASTSerialLib::SetClientAddress(unsigned char ClientAddr)
{
    return SC->SetDevAddr(ClientAddr)==0?-1:0;
}

/**
 * Sends the ping command and waits for the RVS ack.
 *
 * \param DestDevAddr Device address on the serial bus.
 * \return Returns -1 if an error occurs, 0 instead.
**/
int ASTSerialLib::ASTCmd_Ping(unsigned char DestDevAddr)
{
    if(SC->SendSerialCmd(DestDevAddr, CMD_Ping)==-1){
        printf("send serial error\n");
        return -1;
    }

    return SC->ReceiveSerialCmdReply(DestDevAddr, CMD_Ping, 2000);
}

/**
 * Receives the presentation string not larger than MaxLen bytes and stores it in Buffer.
 *
 * \param DestDevAddr Device address on the serial bus.
 * \param MaxLen Max length of the presentation string.
 * \param Buffer Unsigned char buffer where to copy the presentation string.
 * \return Returns the size of the presentation string or -1 if an error occurs.
**/
int ASTSerialLib::ASTCmd_GetPres(unsigned char DestDevAddr, int MaxLen, unsigned char *Buffer)
{
    if(SC->SendSerialCmd(DestDevAddr, CMD_Read_PresString)==-1)
        return -1;
    return SC->ReceiveSerialCmdReply(DestDevAddr, CMD_Read_PresString, CMD_Default_WAIT, MaxLen, Buffer);
}

/**
 * Sets Date and Time on the Host.
 *
 * \param DestDevAddr Device address on the serial bus.
 * \param DateTime pointer to a DateTimeTypeDef structure.
 * \return Returns -1 if an error occurs, 0 instead.
**/
int ASTSerialLib::ASTCmd_SetDateTime(unsigned char DestDevAddr, const DateTimeTypeDef &DateTime)
{
    if(SC->SendSerialCmd(DestDevAddr, CMD_Set_DateTime, sizeof(DateTimeTypeDef), (u8*)&DateTime)==-1)
        return -1;
    return SC->ReceiveSerialCmdReply(DestDevAddr, CMD_Set_DateTime, CMD_Default_WAIT);
}

/**
 * Sets Date and Time on the Host.
 *
 * \param DestDevAddr Device address on the serial bus.
 * \param DateTime pointer to a DateTimeTypeDef structure.
 * \return Returns -1 if an error occurs, 0 instead.
**/
int ASTSerialLib::ASTCmd_GetDateTime(unsigned char DestDevAddr, DateTimeTypeDef *DateTime)
{
    if(SC->SendSerialCmd(DestDevAddr, CMD_Get_DateTime)==-1)
        return -1;
    return SC->ReceiveSerialCmdReply(DestDevAddr, CMD_Get_DateTime, CMD_Default_WAIT, sizeof(DateTimeTypeDef), (u8*)DateTime);
}

/**
 * Send a Buffer message.
 *
 * \param My_Buffer Pointer to unsigned char, it contains message to be send.
 * \param len = lenght of My_Buffer
 * \return Returns -1 if errors occur, 0 instead.
**/
int ASTSerialLib::SendBuffer(unsigned char *My_Buffer,int len){
    if(SC->SendBuffer(My_Buffer,len)==0)
        return -1;

    return 0;
}

/**
 * Send a String
 *
 * \param My_Buffer Pointer to unsigned char, it contains message to be send.
 * \param len = lenght of My_Buffer
 * \return Returns -1 if errors occur, 0 instead.
**/
int ASTSerialLib::SendString(char *String)
{
    if(SC->SendString(String)==0)
        return -1;

    return 0;
}

/**
 * Send a String
 *
 * \param My_Buffer Pointer to unsigned char, it contains message to be send.
 * \param len = lenght of My_Buffer
 * \return Returns -1 if errors occur, 0 instead.
**/
int ASTSerialLib::ReceiveString(char *String, int MaxLen)
{
    return SC->ReceiveString(String, MaxLen, 10);
}
