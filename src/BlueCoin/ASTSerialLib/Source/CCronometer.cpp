/************************** (C) COPYRIGHT 2010 STMicroelectronics ********************/
/**
* \file CCronometer.cpp
* \class CCronometer
*
* \brief This class implements a cronometer that works in both Windows and Linux
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

#include "../Include/CCronometer.h"

#ifdef _WIN32
extern "C" {
#include <windows.h>
}
#else
#include <unistd.h>
#endif



CCronometer::CCronometer(void)
{
    Reset();
}

/**
 * Reset the cronometer.
 *
 **/
void CCronometer::Reset(void)
{
#ifdef _WIN32
    LARGE_INTEGER li;
    QueryPerformanceFrequency(&li);
    Frequency = li.QuadPart;
    QueryPerformanceCounter(&li);
    StartTime = li.QuadPart;
    LastTime = StartTime;
#endif
#ifdef __linux__
    gettimeofday(&tv_StartTime, NULL);
    tv_LastTime.tv_sec=tv_StartTime.tv_sec;
    tv_LastTime.tv_usec=tv_StartTime.tv_usec;
#endif
}


/**
 * Compute current time.
 *
 * \return Returns current time.
 **/
double CCronometer::CurrentTime(void)
{
    double App;
#ifdef _WIN32
    LARGE_INTEGER li;
    QueryPerformanceCounter(&li);
    App = (double)(li.QuadPart - StartTime);
    App = App / Frequency;
#endif

#ifdef __linux__
    struct timeval tv_App;

    gettimeofday(&tv_App, NULL);
    App = (double)(tv_App.tv_sec-tv_StartTime.tv_sec) + (double)(tv_App.tv_usec-tv_StartTime.tv_usec)/(double)1000000;
#endif

    return App;
}

/**
 * Compute the partial time with respect to the last time requested.
 * \return Returns current time.
 **/
double CCronometer::PartialTime(void)
{
    double App;

#ifdef _WIN32
    LARGE_INTEGER li;
    QueryPerformanceCounter(&li);
    App = (double)(li.QuadPart - LastTime);
    App /= Frequency;
    LastTime = li.QuadPart;
#endif

#ifdef __linux__
    struct timeval tv_App;

    gettimeofday(&tv_App, NULL);

    App = (double)(tv_App.tv_sec-tv_LastTime.tv_sec) + (double)(tv_App.tv_usec-tv_LastTime.tv_usec)/(double)1000000;

    gettimeofday(&tv_LastTime, NULL);
#endif

    return App;
}


