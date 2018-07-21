cmake_minimum_required (VERSION 3.5.1)
set (CMAKE_CXX_STANDARD 11)

project(serial)
set(EXECUTABLE_OUTPUT_PATH bin/)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)
include_directories(${PROJECT_ROOT}/protocol)

set(SOURCES
        src/serial.cpp
        src/autopilot_interface.cpp
        src/serial_port.cpp
        src/serial_port.h
        src/autopilot_interface.h
        src/SerialPortAdapter.h)
if(UNIX)
    list(APPEND SOURCES src/SerialPortAdapterUnix.cpp
                        src/SerialPortAdapterUnix.h)
elseif(WIN32)
    list(APPEND SOURCES src/SerialPortAdapterWin.cpp
                        src/SerialPortAdapterWin.h)
endif()

    add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
