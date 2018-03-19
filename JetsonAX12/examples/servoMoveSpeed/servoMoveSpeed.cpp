/*
    Example for moving at variable speeds on the Dynamixel AX12-A series servos
    
	Serial:
	GPIO UART: "/dev/ttyTHS0" "/dev/ttyTHS1" "/dev/ttyTHS2"
	USB  UART: "/dev/ttyUSB0"

    Jetson Pins:
    gpio57  or 57,    // J3A1 - Pin 50
	gpio160 or 160,	  // J3A2 - Pin 40	
	gpio161 or 161,    // J3A2 - Pin 43
	gpio162 or 162,    // J3A2 - Pin 46
	gpio163 or 163,    // J3A2 - Pin 49
	gpio164 or 164,    // J3A2 - Pin 52
	gpio165 or 165,    // J3A2 - Pin 55
	gpio166 or 166     // J3A2 - Pin 58
	
	*Moves the servo to a selected position at the desired speed
		Ax12.moveSpeed(ID, 0->1023, 0->1023): 0->360 degrees
*/

#include<iostream>
#include "JetsonAX12.h"

#define ID 1        // ID for singl servo
#define USB 1   	// 1 for GPIO, 0 for USB
#define SEC 1000000 // 1 Second in micro second units for delay
#define MSEC 1000	// 1 milli second in micro second units for delay

using namespace std;

int main()
{
    JetsonAX12 control;

#if USB
	control.begin("/dev/ttyUSB0", B1000000);
#else 
	control.begin("/dev/ttyTHS0", B1000000, 166);
#endif
    
    control.setEndless(ID, OFF);
    
    for(int i = 0; i < 3; i++)
    {
		control.moveSpeed(ID, 0, 1023);
		usleep(2*SEC);
		control.moveSpeed(ID, 256, 100);
		usleep(2*SEC);
		control.moveSpeed(ID, 512, 300);
		usleep(2*SEC);
		control.moveSpeed(ID, 768, 256);
		usleep(2*SEC);
		control.moveSpeed(ID, 1023, 768);
		usleep(2*SEC);
		control.moveSpeed(ID, 768, 256);
		usleep(2*SEC);
		control.moveSpeed(ID, 512, 50);
		usleep(2*SEC);
		control.moveSpeed(ID, 256, 512);
		usleep(2*SEC);

    }
    
    control.disconnect();
    return 0;
}

