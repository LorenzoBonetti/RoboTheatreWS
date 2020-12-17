/************************** (C) COPYRIGHT 2010 STMicroelectronics ********************
* THE PRESENT FIRMWARE/SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
**************************************************************************************/

#ifndef ASTSERIALLIB_H
#define ASTSERIALLIB_H

#include "SerialComm.h"

/**
  * @brief  DateTime structure definition
  */
typedef struct
{
    u8 Hours;    /*!< Specifies the RTC Time Hour.
                    This parameter must be set to a value in the 0-12 range
                    if the RTC_HourFormat_12 is selected or 0-23 range if
                    the RTC_HourFormat_24 is selected. */

    u8 Minutes;  /*!< Specifies the RTC Time Minutes.
                    This parameter must be set to a value in the 0-59 range. */

    u8 Seconds;  /*!< Specifies the RTC Time Seconds.
                    This parameter must be set to a value in the 0-59 range. */

    u8 WeekDay;  /*!< Specifies the RTC Date WeekDay.
                    1 -> Monday ......  7 -> Sunday */

    u8 Date;     /*!< Specifies the RTC Date.
                    This parameter must be set to a value in the 1-31 range. */

    u8 Month;    /*!< Specifies the RTC Date Month.*/


    u8 Year;     /*!< Specifies the RTC Date Year.
                    This parameter must be set to a value in the 0-99 range. */
} DateTimeTypeDef;


class ASTSerialLib
{
protected:
    SerialComm *SC;
    unsigned char MaxPtPerPacket;
    bool LinkActive;

public:
    ASTSerialLib();
    ~ASTSerialLib();
    void SetCOMPortNumber(int PortNumber);
    int Open(int PortSpeed = 9600);
    void SetCom(SerialComm *SC_external);
    SerialComm * GetCom(void);
#ifdef _WIN32
    int COM_IsRS232();
#endif
    int Close();
    void SetEndOfFrame_AT();
    void SetEndOfFrame_ASTprotocol();
    int ASTCmd_Reset(unsigned char DestDevAddr);
    bool IsLinkActive() { return LinkActive; }
    int SetClientAddress(unsigned char ClientAddr);
    int ASTCmd_Ping(unsigned char DestDevAddr);
    int ASTCmd_GetPres(unsigned char DestDevAddr, int MaxLen, unsigned char *Buffer);
    int ASTCmd_SetDateTime(unsigned char DestDevAddr, const DateTimeTypeDef &DateTime);
    int ASTCmd_GetDateTime(unsigned char DestDevAddr, DateTimeTypeDef *DateTime);
    int SendBuffer(unsigned char *My_Buffer,int len);
    int SendString(char *String);
    int ReceiveString(char *String, int MaxLen);

};

#endif // ASTSERIALLIB_H
