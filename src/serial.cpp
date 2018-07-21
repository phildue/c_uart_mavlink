//====================================================================================================//
	// Serial Port Programming using Win32 API in C                                                       //
	// (Reads data from serial port)                                                                      //
	//====================================================================================================//
	
	//====================================================================================================//
	// www.xanthium.in										                                              //
	// Copyright (C) 2014 Rahul.S                                                                         //
	//====================================================================================================//
	
	//====================================================================================================//
	// The Program runs on the PC side and uses Win32 API to communicate with the serial port or          //
	// USB2SERIAL board and reads the data from it.                                                       //       
	//----------------------------------------------------------------------------------------------------//
	// Program runs on the PC side (Windows) and receives a string of characters.                         //            
	// Program uses CreateFile() function to open a connection serial port(COMxx).                        //
	// Program then sets the parameters of Serial Comm like Baudrate,Parity,Stop bits in the DCB struct.  //
	// After setting the Time outs,the Program waits for the reception of string of characters by setting-//
	// -up the  WaitCommEvent().                                                                          //
	// When a character is reeived bythe PC UART WaitCommEvent() returns and the received string is read- //
	// -using ReadFile(); function.The characters are then displayed on the Console.                      //
	//----------------------------------------------------------------------------------------------------// 
	// BaudRate     -> 9600                                                                               //
	// Data formt   -> 8 databits,No parity,1 Stop bit (8N1)                                              //
	// Flow Control -> None                                                                               //
	//----------------------------------------------------------------------------------------------------//
	
	
	//====================================================================================================//
	// Compiler/IDE  :	Microsoft Visual Studio Express 2013 for Windows Desktop(Version 12.0)            //
	//               :  gcc 4.8.1 (MinGW)                                                                 //
	//                                                                                                    //
	// Library       :  Win32 API,windows.h,                                                              //
	// Commands      :  gcc -o USB2SERIAL_Read_W32 USB2SERIAL_Read_W32.c                                  //
	// OS            :	Windows(Windows 7)                                                                //
	// Programmer    :	Rahul.S                                                                           //
	// Date	         :	30-November-2014                                                                  //
	//====================================================================================================//

	//====================================================================================================//
	// Sellecting the COM port Number                                                                     //
	//----------------------------------------------------------------------------------------------------//
	// Use "Device Manager" in Windows to find out the COM Port number allotted to USB2SERIAL converter-  // 
	// -in your Computer and substitute in the  "ComPortName[]" array.                                    //
	//                                                                                                    //
	// for eg:-                                                                                           //
	// If your COM port number is COM32 in device manager(will change according to system)                //
	// then                                                                                               //
	//			char   ComPortName[] = "\\\\.\\COM32";                                                    //
	//====================================================================================================//

	#include <Windows.h>
	#include <stdio.h>
#include "SerialPortAdapterWin.h"

void main(void)
		{
			char  ComPortName[] = "\\\\.\\COM3";  // Name of the Serial port(May Change) to be opened,
#ifdef WIN32
			SerialPortAdapter port = SerialPortAdapterWin();
#elif __linux__
            SerialPortAdapter port = SerialPortAdapterUnix();
#endif
            port.open(ComPortName);
            port.setup(115200, 8, 1, false, false);
            printf("\n\n    Waiting for Data Reception");
            uint8_t ch;
            while(true){
                int n_read = port.read(&ch);
                if (n_read > 0){
                    printf("%c",ch);
                }

            }

           /*------------------------------------ Setting WaitComm() Event   ----------------------------------------*/
			


		
				printf("\n +==========================================+\n");
				//_getch();
		}//End of Main()