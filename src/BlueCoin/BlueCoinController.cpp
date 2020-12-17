//
// Created by Lorenzo on 10/31/2020.
//

#include "BlueCoinController.h"

void BlueCoinController::connectToCOMPort() {
    AudioSL = new AudioModuleSerialLib();
    AudioSL->SetCOMPortNumber(com_N);
    if (AudioSL->Open(COM_BAUDRATE) < 0) {
        printf("baud <0");
    } else{
        std::cout << "BlueCoin connected " <<com_N<<std::endl;
    }

    if (AudioSL->ASTCmd_Ping(Audio_Module_ADDR) < 0) {
        printf("addr<0");
    }


}

BlueCoinController::BlueCoinController(int com_N) {
    this->com_N = com_N;
}

int BlueCoinController::getAngle() {
    AudioStatusInstance.GeneralStatus.AlgorithmActivation = ALGO_ACTIVATION_SL;
    if (AudioSL->AudioModuleCmd_SetStatus(Audio_Module_ADDR, DOMAIN_GENERAL, &AudioStatusInstance) != 0) {
        throw new SetStatusException;
    }
    if (AudioSL->AudioModuleCmd_GetStatus(Audio_Module_ADDR, DOMAIN_SLOC, &AudioStatusInstance) != 0) {
        throw new GetStatusException;
    } else {
        return (int)AudioStatusInstance.SLocStatus.Angle;
    }
}