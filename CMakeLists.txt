cmake_minimum_required (VERSION 3.5.1)
set (CMAKE_CXX_STANDARD 11)

project(mavlink_test)
set(EXECUTABLE_OUTPUT_PATH ../bin/)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)
include_directories(mavlink/)

set(SOURCES
        #src/serial.cpp
        src/MavlinkControl.cpp
        src/MavlinkControl.h
        src/MavlinkInterface.cpp
        src/SerialPort.cpp
        src/SerialPort.h
        src/MavlinkInterface.h
        src/SerialPortAdapter.h)
if(UNIX)
    list(APPEND SOURCES src/SerialPortAdapterUnix.cpp
                        src/SerialPortAdapterUnix.h)
elseif(WIN32)
    list(APPEND SOURCES src/SerialPortAdapterWin.cpp
                        src/SerialPortAdapterWin.h)
endif()

    add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
