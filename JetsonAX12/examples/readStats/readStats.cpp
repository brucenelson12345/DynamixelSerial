/*
    Example for reading to the Dynamixel AX12-A series servos
    
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
	
	*Reads the temperature of the selected servo
		Ax12.readPosition(ID)
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
    
    control.move(ID, 2048);
    
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

