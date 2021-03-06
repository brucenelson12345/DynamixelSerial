/*
    Example for moving the Dynamixel AX12-A series servos
    
	Serial:
	USB  UART: "/dev/ttyUSB0"
	
	*Moves the servo to the selected position at max speed
		UAX.move(ID, 0->1023): 0->300 degrees
*/

#include<iostream>
#include "UsbAX.h"

#define ID 1        // ID for singl servo
#define SEC 1000000 // 1 Second in micro second units for delay
#define MSEC 1000	// 1 milli second in micro second units for delay

using namespace std;

int main()
{
    UsbAX control;

	control.begin("/dev/ttyUSB0", B1000000);

	control.setEndless(ID, OFF); // Sets the servo to "Servo" mode
    
    for(int i = 0; i < 3; i ++)
    {
        control.move(ID,0);
        usleep(2*SEC);
        
        control.move(ID,256);
        usleep(2*SEC);
        
        control.move(ID,512);
        usleep(2*SEC);
        
        control.move(ID,768);
        usleep(2*SEC);
        
        control.move(ID,1023);
        usleep(2*SEC);
        
        control.move(ID,768);
        usleep(2*SEC);
        
        control.move(ID,512);
        usleep(2*SEC);
        
        control.move(ID,256);
        usleep(2*SEC);
        
    }
    
    control.disconnect();
    return 0;
}

