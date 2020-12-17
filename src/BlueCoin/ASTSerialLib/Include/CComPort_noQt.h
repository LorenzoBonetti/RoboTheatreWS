/************************** (C) COPYRIGHT 2010 STMicroelectronics ********************
* THE PRESENT FIRMWARE/SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
**************************************************************************************/


#ifndef CCOMPORT_NOQT_H
#define CCOMPORT_NOQT_H

#include "CCronometer.h"

#ifdef _WIN32
#include "shlwapi.h"
#endif

#ifdef __linux__
#include "termios.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/time.h>
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#endif


class CComPort
{
public:
    static const int FBufferMaxLen=2048;

private:

    int FOpened;
    int FPortNumber;
    int FPortSpeed;
    int FPortSize;
    int FPortParity;
    int FPortstopBits;
    unsigned char FEndOfFrame;
    int FBufferLen;
    unsigned char FBuffer[FBufferMaxLen];
#ifdef __linux__
    struct termios newtio,oldtio;
    int fd;
#endif
#ifdef _WIN32
    HANDLE hCom;
#endif



private:
#ifdef _WIN32
    void ctow(WCHAR* Dest, const CHAR* Source);
#endif
    int Read(unsigned char *Buffer, int NumToRead, int *NumReceived);

public:
    CComPort(void);
    ~CComPort(void);

    int Open(int PortSpeed = 9600);
    int Opened();
#ifdef _WIN32
    int IsRS232();
#endif
    int Close();
    void SetPortNumber(int Value);
    void SetPortSpeed(int Value);
    void SetEndOfFrame(unsigned char  Value);

    int Write(unsigned char *Buffer, int NumToWrite, int *NumWritten);
    int ReadWait(unsigned char *Buffer, int NumToRead, int *NumReceived, double MaxWait);
    int ReadFrame(unsigned char *Buffer, int MaxDim, int *FrameAvailable, double MaxWait);
};

#endif

