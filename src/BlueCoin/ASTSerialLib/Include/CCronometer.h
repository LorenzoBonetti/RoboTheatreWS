/************************** (C) COPYRIGHT 2010 STMicroelectronics ********************
* File Name          : CComPort.c
* Author             : AST Robotics group
* Date First Issued  : 23 Mar 2010: Version 1.0
* Description        : This file implements the CommunicationPort functions for Linux/Win32
**************************************************************************************
* History:
* 23 Mar 2010: Version 1.0
**************************************************************************************
 THESE LIBRARIES ARE DISTRIBUTED CONFIDENTIALLY BY THE AUTHOR. IT'S FORBIDDEN TO
 REDISTRIBUTE THEM WITHOUT AUTHOR'S AUTHORISATION. FOR COMMERCIAL USE PLEASE
 CONTACT THE AUTHOR.

 STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT
 OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
 OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
 CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
**************************************************************************************/


#ifndef CRONOMETER_H
#define CRONOMETER_H

#ifdef __linux__
#include <sys/time.h>
#include <stdio.h>
#include <time.h>
#endif


class CCronometer {
private:
        #ifdef _WIN32
        long long Frequency;
        long long StartTime;
        long long LastTime;
        #endif
        #ifdef __linux__
        struct timeval tv_StartTime,tv_LastTime;
        #endif

public:
        CCronometer(void);
        void Reset(void);
        double CurrentTime(void);
        double PartialTime(void);
};

#endif
