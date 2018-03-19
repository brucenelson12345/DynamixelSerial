/*
    Example for moving at variable speeds on the Dynamixel AX12-A series servos
    
	Serial:
	USB  UART: "/dev/ttyUSB0"

	*Moves the servo to the selected position (in bits) at variable speed
	Position (0 to 1023) moves counter-clockwise
		UAX.moveSpeedDeg(ID, Position, Speed):
		@ID - ID of the servo
		@Position - bits from 0 to 1023
		@Speed - speed to move from 0 to 1020 or can use the following constants:
				FULL_SPEED = 1020
				TRIQUARTER = 756
				HALF_SPEED = 512
				QUARTER_SPEED = 256
				STOP = 0
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
		control.moveSpeed(ID, 1023, TRIQUARTER_SPEED);
		usleep(2*SEC);
		control.moveSpeed(ID, 768, QUARTER_SPEED);
		usleep(2*SEC);
		control.moveSpeed(ID, 512, 50);
		usleep(2*SEC);
		control.moveSpeed(ID, 256, HALF_SPEED);
		usleep(2*SEC);

    }
    
    control.disconnect();
    return 0;
}

