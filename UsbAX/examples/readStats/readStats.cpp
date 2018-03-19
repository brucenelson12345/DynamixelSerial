/*
    Example for reading to the Dynamixel UAX-A series servos
    
	Serial:
	USB  UART: "/dev/ttyUSB0"
	
	*Reads the temperature of the selected servo
		UAX.readPosition(ID)	: reads the position
		UAX.readTemperature(ID): reads the temperature
		UAX.readVoltage(ID)	: reads the voltage
		UAX.readSpeed(ID)		: reads the speed
		UAX.readLoad(ID)		: reads the load
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
    
    control.move(ID, 512);
    
    for(int i = 0; i < 3; i ++)
    {
        cout << "POSITION: " << control.readPosition(ID) << endl;
        cout << "TEMPERATURE: " << control.readTemperature(ID) << endl;
        cout << "VOLTAGE: " << control.readVoltage(ID) << endl;
        cout << "SPEED: " << control.readSpeed(ID) << endl;
        cout << "LOAD: " << control.readLoad(ID) << endl << endl;
        usleep(2*SEC);
    }
    
    control.disconnect();
    
    return 0;
}

