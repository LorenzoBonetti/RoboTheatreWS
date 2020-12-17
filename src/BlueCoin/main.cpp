#include <iostream>
#include <fstream>

#include "BlueCoinController.h"
#ifndef CCOMPORT_NO_QT
#include <QThread>
#include <QCoreApplication>
#endif



int main() {
    int com_N = 0;
    int angle;
    std::cout << "Hello, World!" << std::endl;
    BlueCoinController blueCoinController(com_N);
    blueCoinController.connectToCOMPort();
    while (1) {
        printf("Angle: %d\n",blueCoinController.getAngle());
       // sleep(1);
    }
    return 0;
}
