/*
    Example for moving in continous rotation mode on the Dynamixel UAX-AT series servos
    
	Serial:
	USB  UART: "/dev/ttyUSB0"

	*Rotates Clockwise
		UAX.turn(ID, RIGHT, 0->1020) : Limit is 1020
		
	*Rotates Counter-Clockwise
		UAX.turn(ID, LEFT,  0->900) : Limit is 900 (Anything above this for CCW may not work)
	
	*Halts at current position
		UAX.turn(ID, OFF, 0) : Stops the servo
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
    
    control.setEndless(ID, ON);
    
    for(int i = 0; i < 3; i++)
    {
		control.turn(ID, RIGHT, 1020);
		usleep(3*SEC);
		control.turn(ID, LEFT, 512);
		usleep(3*SEC);
		control.turn(ID, OFF, OFF); // Either clockwise or counter clockwise as long as speed is 0
		usleep(3*SEC);
    }    
    
    control.disconnect();
    return 0;
}

