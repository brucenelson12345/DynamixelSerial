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
	
	*Moves the servo to the selected position (in degrees) at variable speed
	Position (150 to 0) moves on the left, 0 stops in the middle, (0 to -150) moves to the right
		Ax12.moveSpeedDeg(ID, -150 to 150, 0->1023): 0->300 degrees
		
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

	control.setEndless(ID, OFF); // Sets the servo to "Servo" mode
    
    for(int i = 0; i < 3; i ++)
    {
        control.moveSpeedDeg(ID, 150, 1023);
        usleep(2*SEC);
        
        control.moveSpeedDeg(ID, 120, 768);
        usleep(2*SEC);
        
        control.moveSpeedDeg(ID, 90, 512);
        usleep(2*SEC);
        
        control.moveSpeedDeg(ID, 60, 256);
        usleep(2*SEC);
        
        control.moveSpeedDeg(ID, 30, 128);
        usleep(2*SEC);
        
        control.moveSpeedDeg(ID, 0, 0);
        usleep(2*SEC);
        
        control.moveSpeedDeg(ID, -30, 128);
        usleep(2*SEC);
        
        control.moveSpeedDeg(ID, -60, 256);
        usleep(2*SEC);
        
        control.moveSpeedDeg(ID, -90, 512);
        usleep(2*SEC);
        
        control.moveSpeedDeg(ID, -120, 768);
        usleep(2*SEC);
        
        control.moveSpeedDeg(ID, -150, 1023);
        usleep(2*SEC);
        
    }
    
    control.disconnect();
    return 0;
}

