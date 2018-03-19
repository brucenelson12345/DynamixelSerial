/*
    Example for moving the Dynamixel AX12-A series servos
    
	Serial:
	USB  UART: "/dev/ttyUSB0"
	
	*Moves the servo to the selected position (in degrees) at max speed
	Position (150 to 0) moves on the left, 0 stops in the middle, (0 to -150) moves to the right
		UAX.moveDeg(ID, -150 to 150): 0->300 degrees
		
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
        control.moveDeg(ID,150);
        usleep(2*SEC);
        
        control.moveDeg(ID,120);
        usleep(2*SEC);
        
        control.moveDeg(ID,90);
        usleep(2*SEC);
        
        control.moveDeg(ID,60);
        usleep(2*SEC);
        
        control.moveDeg(ID,30);
        usleep(2*SEC);
        
        control.moveDeg(ID,0);
        usleep(2*SEC);
        
        control.moveDeg(ID,-30);
        usleep(2*SEC);
        
        control.moveDeg(ID,-60);
        usleep(2*SEC);
        
        control.moveDeg(ID,-90);
        usleep(2*SEC);
        
        control.moveDeg(ID,-120);
        usleep(2*SEC);
        
        control.moveDeg(ID,-150);
        usleep(2*SEC);
        
    }
    
    control.disconnect();
    return 0;
}

