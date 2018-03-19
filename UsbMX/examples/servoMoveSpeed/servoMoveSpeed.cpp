/*
    Example for moving at variable speeds on the Dynamixel MX28-AT series servos
    
	Serial:
	USB  UART: "/dev/ttyUSB0"

	*Moves the servo to the selected position (in bits) at variable speed
	Position (0 to 4095) moves counter-clockwise
		UMX.moveSpeedDeg(ID, Position, Speed):
		@ID - ID of the servo
		@Position - bits from 0 to 4095
		@Speed - speed to move from 0 to 1020 or can use the following constants:
				FULL_SPEED = 1020
				TRIQUARTER = 756
				HALF_SPEED = 512
				QUARTER_SPEED = 256
				STOP = 0
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
    
    control.setEndless(ID, OFF);
    
    for(int i = 0; i < 3; i++)
    {
		control.moveSpeed(ID, 0, 1023);
		usleep(2*SEC);
		control.moveSpeed(ID, 1024, 100);
		usleep(2*SEC);
		control.moveSpeed(ID, 2048, 300);
		usleep(2*SEC);
		control.moveSpeed(ID, 3072, 256);
		usleep(2*SEC);
		control.moveSpeed(ID, 4095, TRIQUARTER_SPEED);
		usleep(2*SEC);
		control.moveSpeed(ID, 3072, QUARTER_SPEED);
		usleep(2*SEC);
		control.moveSpeed(ID, 2048, 50);
		usleep(2*SEC);
		control.moveSpeed(ID, 1024, HALF_SPEED);
		usleep(2*SEC);

    }
    
    control.disconnect();
    return 0;
}

