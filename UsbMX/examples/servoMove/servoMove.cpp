/*
    Example for moving the Dynamixel MX28-AT series servos
    
	Serial:
	USB  UART: "/dev/ttyUSB0"
	
	*Moves the servo to the selected position at max speed
		UMX.move(ID, 0->4095): 0->360 degrees
*/

#include<iostream>
#include "UsbMX.h"

#define ID 1        // ID for singl servo
#define SEC 1000000 // 1 Second in micro second units for delay
#define MSEC 1000	// 1 milli second in micro second units for delay

using namespace std;

int main()
{
    UsbMX control;

	control.begin("/dev/ttyUSB0", B1000000);

	control.setEndless(ID, OFF); // Sets the servo to "Servo" mode
    
    for(int i = 0; i < 3; i ++)
    {
        control.move(ID,0);
        usleep(2*SEC);
        
        control.move(ID,1024);
        usleep(2*SEC);
        
        control.move(ID,2048);
        usleep(2*SEC);
        
        control.move(ID,3072);
        usleep(2*SEC);
        
        control.move(ID,4095);
        usleep(2*SEC);
        
        control.move(ID,3072);
        usleep(2*SEC);
        
        control.move(ID,2048);
        usleep(2*SEC);
        
        control.move(ID,1024);
        usleep(2*SEC);
        
    }
    
    control.disconnect();
    return 0;
}

