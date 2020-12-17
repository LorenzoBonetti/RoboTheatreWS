/**
 *
 ******************************************************************************
 * \mainpage AudioModuleSerialLib Documentation
 * <h2><center> AST Robotics Group
 * <br>
 * <br>
 * v 0.1
 * <br>
 * <br>
 * 07/2013
 * <br>
 * <br>
 * <br>
 </center></h2>
 ******************************************************************************
 * \section STLic ST Software License
 *
 * THESE LIBRARIES ARE DISTRIBUTED CONFIDENTIALLY BY THE AUTHOR. IT'S FORBIDDEN TO
 * REDISTRIBUTE THEM WITHOUT AUTHOR'S AUTHORISATION. FOR COMMERCIAL USE PLEASE
 * CONTACT THE AUTHOR.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT
 * OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
 * OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
 * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
 */


#include "AudioModuleSerialLib.h"
#include "../ASTSerialLib/Include/serial_protocol.h"
#include "string.h"
#include "stdio.h"
#include "stdint.h"


#define MUL (1000.0f)

AudioModuleSerialLib::AudioModuleSerialLib(): ASTSerialLib()
{

}

AudioModuleSerialLib::~AudioModuleSerialLib()
{

}

int AudioModuleSerialLib::AudioModuleCmd_SetStatus(unsigned char DestDevAddr, uint8_t StatusDomain, TAudioStatus * Status)
{

    u8 MsgData[TMsg_MaxLen];
    MsgData[0] = StatusDomain;

    uint8_t * pdata = &MsgData[1];
    uint32_t ByteCount=1;

    if(StatusDomain & DOMAIN_GENERAL)
    {
        memcpy(pdata, &(Status->GeneralStatus), sizeof(TGeneralParam));
        pdata += sizeof(TGeneralParam);
        ByteCount += sizeof(TGeneralParam);
    }

    if(StatusDomain & DOMAIN_SLOC)
    {
        memcpy(pdata, &(Status->SLocStatus), sizeof(TSLocParam));
        pdata += sizeof(TSLocParam);
        ByteCount += sizeof(TSLocParam);
    }

    if(StatusDomain & DOMAIN_BEAMFORMING)
    {
        memcpy(pdata, &(Status->BeamStatus), sizeof(TBeamParam));
        pdata += sizeof(TBeamParam);
        ByteCount += sizeof(TBeamParam);
    }

    if(StatusDomain & DOMAIN_AEC)
    {
        memcpy(pdata, &(Status->AECStatus), sizeof(TAECParam));
        pdata += sizeof(TAECParam);
        ByteCount += sizeof(TAECParam);
    }
    if(StatusDomain & DOMAIN_DBNOISE)
    {
        memcpy(pdata, &(Status->dBStatus), sizeof(TdBParam));
        pdata += sizeof(TdBParam);
        ByteCount += sizeof(TdBParam);
    }

    if(StatusDomain & DOMAIN_ASR)
    {
        memcpy(pdata, &(Status->ASRStatus), sizeof(TASRParam));
        pdata += sizeof(TASRParam);
        ByteCount += sizeof(TASRParam);
    }

    if(StatusDomain & DOMAIN_OUTPUT)
    {
        memcpy(pdata, &(Status->OutputStatus), sizeof(TOutputParam));
        pdata += sizeof(TOutputParam);
        ByteCount += sizeof(TOutputParam);
    }

    if(SC->SendSerialCmd(DestDevAddr, CMD_AudioModule_SetStatus, ByteCount, MsgData)==-1)
        return -1;
    return SC->ReceiveSerialCmdReply(DestDevAddr, CMD_AudioModule_SetStatus, CMD_Default_WAIT);
}

int AudioModuleSerialLib::AudioModuleCmd_GetStatus(unsigned char DestDevAddr, uint8_t StatusDomain, TAudioStatus *Status)
{

    int ret=-1;
    u8 OutData[sizeof(TAudioStatus)+16]={0};
    uint8_t * pdata= &OutData[1];

    if(SC->SendSerialCmd(DestDevAddr, CMD_AudioModule_GetStatus, 1, &StatusDomain)==-1)
        return -1;

    ret = SC->ReceiveSerialCmdReply(DestDevAddr, CMD_AudioModule_GetStatus, CMD_Default_WAIT, sizeof(TAudioStatus)+16, (u8 *) OutData);

    /*Due to the added domain byte in the message*/
    int expected_datalen = 1;

    if(StatusDomain & DOMAIN_GENERAL)
    {
        memcpy(&(Status->GeneralStatus), pdata, sizeof(TGeneralParam));
        pdata += sizeof(TGeneralParam);
        expected_datalen += sizeof(TGeneralParam);
    }

    if(StatusDomain & DOMAIN_SLOC)
    {
        memcpy(&(Status->SLocStatus), pdata, sizeof(TSLocParam));
        pdata += sizeof(TSLocParam);
        expected_datalen += sizeof(TSLocParam);

    }

    if(StatusDomain & DOMAIN_BEAMFORMING)
    {
        memcpy(&(Status->BeamStatus), pdata, sizeof(TBeamParam));
       pdata += sizeof(TBeamParam);
       expected_datalen += sizeof(TBeamParam);

    }

    if(StatusDomain & DOMAIN_AEC)
    {
        memcpy(&(Status->AECStatus), pdata, sizeof(TAECParam));
        pdata += sizeof(TAECParam);
        expected_datalen += sizeof(TAECParam);


    }
    if(StatusDomain & DOMAIN_DBNOISE)
    {
        memcpy(&(Status->dBStatus), pdata, sizeof(TdBParam));
        pdata += sizeof(TdBParam);
        expected_datalen += sizeof(TdBParam);

    }

    if(StatusDomain & DOMAIN_ASR)
    {
        memcpy(&(Status->ASRStatus), pdata, sizeof(TASRParam));
        pdata += sizeof(TASRParam);
        expected_datalen += sizeof(TASRParam);

    }

    if(StatusDomain & DOMAIN_OUTPUT)
    {
        memcpy(&(Status->OutputStatus), pdata, sizeof(TOutputParam));
        pdata += sizeof(TOutputParam);
        expected_datalen += sizeof(TOutputParam);

    }

    if (ret == expected_datalen)
    {
        return 0;
    }
    else
    {
        return -1;
    }
}


