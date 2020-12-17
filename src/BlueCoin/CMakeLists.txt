cmake_minimum_required(VERSION 3.16)
project(BlueCoin)

set(CMAKE_CXX_STANDARD 14)
add_subdirectory(ASTSerialLib)
add_subdirectory(AudioSerialLib)
add_executable(BlueCoin main.cpp BlueCoinController.cpp BlueCoinController.h Exceptions.cpp)

target_link_libraries(BlueCoin PUBLIC ASTSerialLib)
target_link_libraries(BlueCoin PUBLIC AudioSerialLib)


