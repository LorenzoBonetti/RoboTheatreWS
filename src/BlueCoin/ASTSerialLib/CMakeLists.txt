cmake_minimum_required(VERSION 3.16)
project(BlueCoin)

set(CMAKE_CXX_STANDARD 14)
include_directories(Include Source)
add_library(ASTSerialLib Include/ASTSerialLib.h Include/CComPort_noQt.h Include/CCronometer.h Include/Serial_CMD.h Include/serial_protocol.h Include/SerialComm.h Source/ASTSerialLib.cpp Source/CComPort_noQt.cpp Source/CCronometer.cpp  Source/serial_protocol.c Source/SerialComm.cpp )