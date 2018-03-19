/*
********************************************************************************************
    Library for Dynamixel AX12A on the Jetson TK1
    
    The purpose of this library is to simplify operation on the Jetson TK1 microcontroller
    through either the GPIO UART or USB UART. 
    
    Special thanks to Josue Alejandro Savage for the Dynamixel Arduino library for
    which this library was derived from as well as the Dynamixel SDK. In addition to
    Jetson Hacks for providing the Jetson TK1 GPIO library.
    
    MODIFICATIONS:
    2/18/2018 - Created the library with read and write functions
    2/23/2018 - Added the USB UART comptability
    
    TODO:
    - Fix Read functions to packets in order and to avoid faulty packets
    - Fix Read to read data majority of the time
    
********************************************************************************************

AUTHOR: Bruce Nelson
ORGANIZATION: Sparta Robotics

*/

#include "JetsonAX12.h"

void JetsonAX12::begin(const char *stream, speed_t baud, jetsonGPIO dataPin)
{
    // Configure GPIO
    gpio_status = ON;
    data = dataPin;
    gpioExport(data);
    gpioSetDirection(data,outputPin);
    
    // Configure UART
    uart0_filestream = -1;
    uart0_filestream = open("/dev/ttyTHS0", O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode
	if (uart0_filestream == -1)
	{
		//ERROR - CAN'T OPEN SERIAL PORT
		printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
	}
	
	struct termios options;
	tcgetattr(uart0_filestream, &options);
	options.c_cflag = baud | CS8 | CLOCAL | CREAD;		//<Set baud rate
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(uart0_filestream, TCIFLUSH);
	tcsetattr(uart0_filestream, TCSANOW, &options);
}

void JetsonAX12::begin(const char *stream, speed_t baud)
{
    // Configure GPIO
    gpio_status = OFF;
    
    uart0_filestream = open(stream, O_RDWR| O_NOCTTY );
    
    struct termios tty;
    struct termios tty_old;
    memset (&tty, 0, sizeof tty);

    if ( tcgetattr ( uart0_filestream, &tty ) != 0 ) {
       std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
    }

    tty_old = tty;

    cfsetospeed (&tty, baud);
    cfsetispeed (&tty, baud);

    tty.c_cflag     &=  ~PARENB;
    tty.c_cflag     &=  ~CSTOPB;
    tty.c_cflag     &=  ~CSIZE;
    tty.c_cflag     |=  CS8;

    tty.c_cflag     &=  ~CRTSCTS;           // no flow control
    tty.c_cc[VMIN]   =  1;                  // read doesn't block
    tty.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
    tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

    cfmakeraw(&tty);

    tcflush( uart0_filestream, TCIFLUSH );
    if ( tcsetattr ( uart0_filestream, TCSANOW, &tty ) != 0) {
       std::cout << "Error " << errno << " from tcsetattr" << std::endl;
    }
}

void JetsonAX12::disconnect()
{
	if(gpio_status)
		gpioUnexport(data);
		
    close(uart0_filestream);
    printf("DISCONNECTED!!! \n");
}

int JetsonAX12::reset(unsigned char ID)
{
    memset(tx_buffer, 0, sizeof(tx_buffer) );

	Checksum = (~(ID + AX_RESET_LENGTH + AX_RESET))&0xFF;
    
	TRANSMIT_ON(gpio_status);
    tx_buffer[0] = AX_START;
    tx_buffer[1] = AX_START;
    tx_buffer[2] = ID;
    tx_buffer[3] = AX_RESET_LENGTH;
    tx_buffer[4] = AX_RESET;
    tx_buffer[5] = Checksum;
    
	count = write(uart0_filestream, &tx_buffer, 6);
	if (count < 0)
	{
		printf("UART TX error\n");
	}
	
	usleep(TX_DELAY_TIME);
	TRANSMIT_OFF(gpio_status);

    return 0;
}

int JetsonAX12::ping(unsigned char ID)
{
    memset(tx_buffer, 0, sizeof(tx_buffer) );

	Checksum = (~(ID + AX_READ_DATA + AX_PING))&0xFF;
    
	TRANSMIT_ON(gpio_status);
    tx_buffer[0] = AX_START;
    tx_buffer[1] = AX_START;
    tx_buffer[2] = ID;
    tx_buffer[3] = AX_PING;
    tx_buffer[4] = Checksum;
    
	count = write(uart0_filestream, &tx_buffer, 5);
	if (count < 0)
	{
		printf("UART TX error\n");
	}
	
	usleep(TX_DELAY_TIME);
	TRANSMIT_OFF(gpio_status);

    return 0;
}

int JetsonAX12::setID(unsigned char ID, unsigned char newID)
{
    memset(tx_buffer, 0, sizeof(tx_buffer) );

	Checksum = (~(ID + AX_ID_LENGTH + AX_WRITE_DATA + AX_ID + newID))&0xFF;
    
	TRANSMIT_ON(gpio_status);
    tx_buffer[0] = AX_START;
    tx_buffer[1] = AX_START;
    tx_buffer[2] = ID;
    tx_buffer[3] = AX_ID_LENGTH;
    tx_buffer[4] = AX_WRITE_DATA;
    tx_buffer[5] = AX_ID;
    tx_buffer[6] = newID;
    tx_buffer[7] = Checksum;
    
	count = write(uart0_filestream, &tx_buffer, 8);
	if (count < 0)
	{
		printf("UART TX error\n");
	}
	
	usleep(TX_DELAY_TIME);
	TRANSMIT_OFF(gpio_status);

    return 0;
}

int JetsonAX12::setBD(unsigned char ID, long baud)
{
    unsigned char Baud_Rate = (2000000/baud) - 1;
    
    memset(tx_buffer, 0, sizeof(tx_buffer) );

	Checksum = (~(ID + AX_BD_LENGTH + AX_WRITE_DATA + AX_BAUD_RATE + Baud_Rate))&0xFF;
    
	TRANSMIT_ON(gpio_status);
    tx_buffer[0] = AX_START;
    tx_buffer[1] = AX_START;
    tx_buffer[2] = ID;
    tx_buffer[3] = AX_BD_LENGTH;
    tx_buffer[4] = AX_WRITE_DATA;
    tx_buffer[5] = AX_BAUD_RATE;
    tx_buffer[6] = Baud_Rate;
    tx_buffer[7] = Checksum;
    
	count = write(uart0_filestream, &tx_buffer, 8);
	if (count < 0)
	{
		printf("UART TX error\n");
	}
	
	usleep(TX_DELAY_TIME);
	TRANSMIT_OFF(gpio_status);

    return 0;
}

int JetsonAX12::move(unsigned char ID, int Position)
{
    char Position_H,Position_L;
    
    memset(tx_buffer, 0, sizeof(tx_buffer) );
   
    Position_H = Position >> 8;
    Position_L = Position;
	Checksum = (~(ID + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_GOAL_POSITION_L + Position_L + Position_H))&0xFF;
    
	TRANSMIT_ON(gpio_status);
    tx_buffer[0] = AX_START;
    tx_buffer[1] = AX_START;
    tx_buffer[2] = ID;
    tx_buffer[3] = AX_GOAL_LENGTH;
    tx_buffer[4] = AX_WRITE_DATA;
    tx_buffer[5] = AX_GOAL_POSITION_L;
    tx_buffer[6] = Position_L;
    tx_buffer[7] = Position_H;
    tx_buffer[8] = Checksum;
    
	count = write(uart0_filestream, &tx_buffer, 9);
	if (count < 0)
	{
		printf("UART TX error\n");
	}
	
	usleep(TX_DELAY_TIME);
	TRANSMIT_OFF(gpio_status);

    return 0;
}

int JetsonAX12::moveSpeed(unsigned char ID, int Position, int Speed)
{
    char Position_H,Position_L,Speed_H,Speed_L;
    
    memset(tx_buffer, 0, sizeof(tx_buffer) );
   
    Position_H = Position >> 8;
    Position_L = Position;
    Speed_H = Speed >> 8;
    Speed_L = Speed;
	Checksum = (~(ID + AX_GOAL_SP_LENGTH + AX_WRITE_DATA + AX_GOAL_POSITION_L + Position_L + Position_H + Speed_L + Speed_H))&0xFF;
    
	TRANSMIT_ON(gpio_status);
    tx_buffer[0] = AX_START;
    tx_buffer[1] = AX_START;
    tx_buffer[2] = ID;
    tx_buffer[3] = AX_GOAL_SP_LENGTH;
    tx_buffer[4] = AX_WRITE_DATA;
    tx_buffer[5] = AX_GOAL_POSITION_L;
    tx_buffer[6] = Position_L;
    tx_buffer[7] = Position_H;
    tx_buffer[8] = Speed_L;
    tx_buffer[9] = Speed_H;
    tx_buffer[10] = Checksum;
    
	count = write(uart0_filestream, &tx_buffer, 11);
	if (count < 0)
	{
		printf("UART TX error\n");
	}
	
	usleep(TX_DELAY_TIME);
	TRANSMIT_OFF(gpio_status);

    return 0;
}

int JetsonAX12::moveDeg(unsigned char ID, int Degrees)
{
	int Position;
	
	Degrees += (CENTER);
	Position = ( float(Degrees) / 300) * 1023;
	
	move(ID, Position);
	
	return 0;
}

int JetsonAX12::moveSpeedDeg(unsigned char ID, int Degrees, int Speed)
{
	int Position;
	
	Degrees += CENTER;
	Position = ( float(Degrees) / 300) * 1023;
	
	moveSpeed(ID, Position, Speed);
	
	return 0;
}

int JetsonAX12::setEndless(unsigned char ID,bool Status)
{

    if ( Status ) {	// for continous mode
	    char AX_CCW_AL_LT = 0;
	    Checksum = (~(ID + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_CCW_ANGLE_LIMIT_L))&0xFF;
    
        memset(tx_buffer, 0, sizeof(tx_buffer) );
        
	    TRANSMIT_ON(gpio_status);
        tx_buffer[0] = AX_START;
        tx_buffer[1] = AX_START;
        tx_buffer[2] = ID;
        tx_buffer[3] = AX_GOAL_LENGTH;
        tx_buffer[4] = AX_WRITE_DATA;
        tx_buffer[5] = AX_CCW_ANGLE_LIMIT_L;
        tx_buffer[6] = AX_CCW_AL_LT;
        tx_buffer[7] = AX_CCW_AL_LT;
        tx_buffer[8] = Checksum;
        
	    count = write(uart0_filestream, &tx_buffer, 9);
	    if (count < 0)
	    {
		    printf("UART TX error\n");
	    }
	
	    usleep(TX_DELAY_TIME);
	    TRANSMIT_OFF(gpio_status);

        return 0;
    
    }
    else // for servo mode
    {
	    turn(ID,0,0);
	    Checksum = (~(ID + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_CCW_ANGLE_LIMIT_L + AX_CCW_AL_L + AX_CCW_AL_H))&0xFF;
    
        memset(tx_buffer, 0, sizeof(tx_buffer) );
        
	    TRANSMIT_ON(gpio_status);

        tx_buffer[0] = AX_START;
        tx_buffer[1] = AX_START;
        tx_buffer[2] = ID;
        tx_buffer[3] = AX_GOAL_LENGTH;
        tx_buffer[4] = AX_WRITE_DATA;
        tx_buffer[5] = AX_CCW_ANGLE_LIMIT_L;
        tx_buffer[6] = AX_CCW_AL_L;
        tx_buffer[7] = AX_CCW_AL_H;
        tx_buffer[8] = Checksum;
        
	    count = write(uart0_filestream, &tx_buffer, 9);
	    if (count < 0)
	    {
		    printf("UART TX error\n");
	    }
	
	    usleep(TX_DELAY_TIME);
	    TRANSMIT_OFF(gpio_status);

        return 0;
    }
}

int JetsonAX12::turn(unsigned char ID, bool SIDE, int Speed)
{
    
	if (SIDE == 0){
		char Speed_H,Speed_L;
		Speed_H = Speed >> 8;
		Speed_L = Speed;
		Checksum = (~(ID + AX_SPEED_LENGTH + AX_WRITE_DATA + AX_GOAL_SPEED_L + Speed_L + Speed_H))&0xFF;
    
	    TRANSMIT_ON(gpio_status);
        tx_buffer[0] = AX_START;
        tx_buffer[1] = AX_START;
        tx_buffer[2] = ID;
        tx_buffer[3] = AX_SPEED_LENGTH;
        tx_buffer[4] = AX_WRITE_DATA;
        tx_buffer[5] = AX_GOAL_SPEED_L;
        tx_buffer[6] = Speed_L;
        tx_buffer[7] = Speed_H;
        tx_buffer[8] = Checksum;
        
	    count = write(uart0_filestream, &tx_buffer, 9);
	    if (count < 0)
	    {
		    printf("UART TX error\n");
	    }
	
	    usleep(TX_DELAY_TIME);
	    TRANSMIT_OFF(gpio_status);

        return 0;
	}
	else
	{
		char Speed_H,Speed_L;
		Speed_H = (Speed >> 8) + 4;
		Speed_L = Speed;
		Checksum = (~(ID + AX_SPEED_LENGTH + AX_WRITE_DATA + AX_GOAL_SPEED_L + Speed_L + Speed_H))&0xFF;
    
	    TRANSMIT_ON(gpio_status);
        tx_buffer[0] = AX_START;
        tx_buffer[1] = AX_START;
        tx_buffer[2] = ID;
        tx_buffer[3] = AX_SPEED_LENGTH;
        tx_buffer[4] = AX_WRITE_DATA;
        tx_buffer[5] = AX_GOAL_SPEED_L;
        tx_buffer[6] = Speed_L;
        tx_buffer[7] = Speed_H;
        tx_buffer[8] = Checksum;
        
	    count = write(uart0_filestream, &tx_buffer, 9);
	    if (count < 0)
	    {
		    printf("UART TX error\n");
	    }
	
	    usleep(TX_DELAY_TIME);
	    TRANSMIT_OFF(gpio_status);

        return 0;
		}
}

int JetsonAX12::moveRW(unsigned char ID, int Position)
{

    char Position_H,Position_L;
    
    Position_H = Position >> 8;
    Position_L = Position;
    
    Checksum = (~(ID + AX_GOAL_LENGTH + AX_REG_WRITE + AX_GOAL_POSITION_L + Position_L + Position_H))&0xFF;
        
	TRANSMIT_ON(gpio_status);
    tx_buffer[0] = AX_START;
    tx_buffer[1] = AX_START;
    tx_buffer[2] = ID;
    tx_buffer[3] = AX_GOAL_LENGTH;
    tx_buffer[4] = AX_REG_WRITE;
    tx_buffer[5] = AX_GOAL_POSITION_L;
    tx_buffer[6] = Position_L;
    tx_buffer[7] = Position_H;
    tx_buffer[8] = Checksum;
    
	count = write(uart0_filestream, &tx_buffer, 9);
	if (count < 0)
	{
		printf("UART TX error\n");
	}
	
	usleep(TX_DELAY_TIME);
	TRANSMIT_OFF(gpio_status);

    return 0;
}

int JetsonAX12::moveSpeedRW(unsigned char ID, int Position, int Speed)
{

    char Position_H,Position_L,Speed_H,Speed_L;
    
    memset(tx_buffer, 0, sizeof(tx_buffer) );
   
    Position_H = Position >> 8;
    Position_L = Position;
    Speed_H = Speed >> 8;
    Speed_L = Speed;
	Checksum = (~(ID + AX_GOAL_SP_LENGTH + AX_REG_WRITE + AX_GOAL_POSITION_L + Position_L + Position_H + Speed_L + Speed_H))&0xFF;
    
	TRANSMIT_ON(gpio_status);
    tx_buffer[0] = AX_START;
    tx_buffer[1] = AX_START;
    tx_buffer[2] = ID;
    tx_buffer[3] = AX_GOAL_SP_LENGTH;
    tx_buffer[4] = AX_REG_WRITE;
    tx_buffer[5] = AX_GOAL_POSITION_L;
    tx_buffer[6] = Position_L;
    tx_buffer[7] = Position_H;
    tx_buffer[8] = Speed_L;
    tx_buffer[9] = Speed_H;
    tx_buffer[10] = Checksum;
    
	count = write(uart0_filestream, &tx_buffer, 11);
	if (count < 0)
	{
		printf("UART TX error\n");
	}
	
	usleep(TX_DELAY_TIME);
	TRANSMIT_OFF(gpio_status);

    return 0;
}

void JetsonAX12::action()
{
    
	TRANSMIT_ON(gpio_status);
    tx_buffer[0] = AX_START;
    tx_buffer[1] = AX_START;
    tx_buffer[2] = BROADCAST_ID;
    tx_buffer[3] = AX_ACTION_LENGTH;
    tx_buffer[4] = AX_ACTION;
    tx_buffer[5] = AX_ACTION_CHECKSUM;
    
	count = write(uart0_filestream, &tx_buffer, 6);
	if (count < 0)
	{
		printf("UART TX error\n");
	}
	
	usleep(TX_DELAY_TIME);
	TRANSMIT_OFF(gpio_status);
}

int JetsonAX12::torqueStatus( unsigned char ID, bool Status)
{
    
    Checksum = (~(ID + AX_TORQUE_LENGTH + AX_WRITE_DATA + AX_TORQUE_ENABLE + Status))&0xFF;
    
	TRANSMIT_ON(gpio_status);
    tx_buffer[0] = AX_START;
    tx_buffer[1] = AX_START;
    tx_buffer[2] = ID;
    tx_buffer[3] = AX_TORQUE_LENGTH;
    tx_buffer[4] = AX_WRITE_DATA;
    tx_buffer[5] = AX_TORQUE_ENABLE;
    tx_buffer[6] = Status;
    tx_buffer[7] = Checksum;
    
	count = write(uart0_filestream, &tx_buffer, 8);
	if (count < 0)
	{
		printf("UART TX error\n");
	}
	
	usleep(TX_DELAY_TIME);
	TRANSMIT_OFF(gpio_status);

    return 0;
}

int JetsonAX12::ledStatus( unsigned char ID, bool Status)
{
    
    Checksum = (~(ID + AX_LED_LENGTH + AX_WRITE_DATA + AX_LED + Status))&0xFF;
    
	TRANSMIT_ON(gpio_status);
    tx_buffer[0] = AX_START;
    tx_buffer[1] = AX_START;
    tx_buffer[2] = ID;
    tx_buffer[3] = AX_LED_LENGTH;
    tx_buffer[4] = AX_WRITE_DATA;
    tx_buffer[5] = AX_LED;
    tx_buffer[6] = Status;
    tx_buffer[7] = Checksum;
    
	count = write(uart0_filestream, &tx_buffer, 8);
	if (count < 0)
	{
		printf("UART TX error\n");
	}
	
	usleep(TX_DELAY_TIME);
	TRANSMIT_OFF(gpio_status);

    return 0;
}

int JetsonAX12::setTempLimit(unsigned char ID, unsigned char Temperature)
{
    
    Checksum = (~(ID + AX_TL_LENGTH +AX_WRITE_DATA+ AX_LIMIT_TEMPERATURE + Temperature))&0xFF;
    
	TRANSMIT_ON(gpio_status);
    tx_buffer[0] = AX_START;
    tx_buffer[1] = AX_START;
    tx_buffer[2] = ID;
    tx_buffer[3] = AX_TL_LENGTH;
    tx_buffer[4] = AX_WRITE_DATA;
    tx_buffer[5] = AX_LIMIT_TEMPERATURE;
    tx_buffer[6] = Temperature;
    tx_buffer[7] = Checksum;
    
	count = write(uart0_filestream, &tx_buffer, 8);
	if (count < 0)
	{
		printf("UART TX error\n");
	}
	
	usleep(TX_DELAY_TIME);
	TRANSMIT_OFF(gpio_status);

    return 0;
}

int JetsonAX12::setVoltageLimit(unsigned char ID, unsigned char DVoltage, unsigned char UVoltage)
{
    
    Checksum = (~(ID + AX_VL_LENGTH +AX_WRITE_DATA+ AX_DOWN_LIMIT_VOLTAGE + DVoltage + UVoltage))&0xFF;
    
	TRANSMIT_ON(gpio_status);
    tx_buffer[0] = AX_START;
    tx_buffer[1] = AX_START;
    tx_buffer[2] = ID;
    tx_buffer[3] = AX_VL_LENGTH;
    tx_buffer[4] = AX_WRITE_DATA;
    tx_buffer[5] = AX_DOWN_LIMIT_VOLTAGE;
    tx_buffer[6] = DVoltage;
	tx_buffer[7] = UVoltage;
    tx_buffer[8] = Checksum;
    
	count = write(uart0_filestream, &tx_buffer, 9);
	if (count < 0)
	{
		printf("UART TX error\n");
	}
	
	usleep(TX_DELAY_TIME);
	TRANSMIT_OFF(gpio_status);

    return 0;
}

int JetsonAX12::setAngleLimit(unsigned char ID, int CWLimit, int CCWLimit)
{
    char CW_H,CW_L,CCW_H,CCW_L;
    
    CW_H = CWLimit >> 8;    
    CW_L = CWLimit;
    CCW_H = CCWLimit >> 8;
    CCW_L = CCWLimit;  
    
	Checksum = (~(ID + AX_VL_LENGTH +AX_WRITE_DATA+ AX_CW_ANGLE_LIMIT_L + CW_H + CW_L + AX_CCW_ANGLE_LIMIT_L + CCW_H + CCW_L))&0xFF;
    
	TRANSMIT_ON(gpio_status);
    tx_buffer[0] = AX_START;
    tx_buffer[1] = AX_START;
    tx_buffer[2] = ID;
    tx_buffer[3] = AX_CCW_CW_LENGTH;
    tx_buffer[4] = AX_WRITE_DATA;
    tx_buffer[5] = AX_CW_ANGLE_LIMIT_L;
    tx_buffer[6] = CW_L;
    tx_buffer[7] = CW_H;
    tx_buffer[8] = AX_CCW_ANGLE_LIMIT_L;
    tx_buffer[9] = CCW_L;
    tx_buffer[10] = CCW_H;
    tx_buffer[11] = Checksum;
    
	count = write(uart0_filestream, &tx_buffer, 12);
	if (count < 0)
	{
		printf("UART TX error\n");
	}
	
	usleep(TX_DELAY_TIME);
	TRANSMIT_OFF(gpio_status);

    return 0;
}

int JetsonAX12::setMaxTorque(unsigned char ID, int MaxTorque)
{
    char MaxTorque_H,MaxTorque_L;
    
    MaxTorque_H = MaxTorque >> 8;
    MaxTorque_L = MaxTorque;
    
	Checksum = (~(ID + AX_MT_LENGTH + AX_WRITE_DATA + AX_MAX_TORQUE_L + MaxTorque_L + MaxTorque_H))&0xFF;
    
	TRANSMIT_ON(gpio_status);
    tx_buffer[0] = AX_START;
    tx_buffer[1] = AX_START;
    tx_buffer[2] = ID;
    tx_buffer[3] = AX_MT_LENGTH;
    tx_buffer[4] = AX_WRITE_DATA;
    tx_buffer[5] = AX_MAX_TORQUE_L;
    tx_buffer[6] = MaxTorque_L;
    tx_buffer[7] = MaxTorque_H;
    tx_buffer[8] = Checksum;
    
	count = write(uart0_filestream, &tx_buffer, 9);
	if (count < 0)
	{
		printf("UART TX error\n");
	}
	
	usleep(TX_DELAY_TIME);
	TRANSMIT_OFF(gpio_status);

    return 0;
}

int JetsonAX12::setSRL(unsigned char ID, unsigned char SRL)
{
    
    Checksum = (~(ID + AX_SRL_LENGTH + AX_WRITE_DATA + AX_RETURN_LEVEL + SRL))&0xFF;
    
	TRANSMIT_ON(gpio_status);
    tx_buffer[0] = AX_START;
    tx_buffer[1] = AX_START;
    tx_buffer[2] = ID;
    tx_buffer[3] = AX_SRL_LENGTH;
    tx_buffer[4] = AX_WRITE_DATA;
    tx_buffer[5] = AX_RETURN_LEVEL;
    tx_buffer[6] = SRL;
    tx_buffer[7] = Checksum;
    
	count = write(uart0_filestream, &tx_buffer, 8);
	if (count < 0)
	{
		printf("UART TX error\n");
	}
	
	usleep(TX_DELAY_TIME);
	TRANSMIT_OFF(gpio_status);

    return 0;
}

int JetsonAX12::setRDT(unsigned char ID, unsigned char RDT)
{
    
   Checksum = (~(ID + AX_RDT_LENGTH + AX_WRITE_DATA + AX_RETURN_DELAY_TIME + (RDT/2)))&0xFF;
    
	TRANSMIT_ON(gpio_status);
    tx_buffer[0] = AX_START;
    tx_buffer[1] = AX_START;
    tx_buffer[2] = ID;
    tx_buffer[3] = AX_RDT_LENGTH;
    tx_buffer[4] = AX_WRITE_DATA;
    tx_buffer[5] = AX_RETURN_DELAY_TIME;
    tx_buffer[6] = RDT/2;
    tx_buffer[7] = Checksum;
    
	count = write(uart0_filestream, &tx_buffer, 8);
	if (count < 0)
	{
		printf("UART TX error\n");
	}
	
	usleep(TX_DELAY_TIME);
	TRANSMIT_OFF(gpio_status);

    return 0;
}

int JetsonAX12::setLEDAlarm(unsigned char ID, unsigned char LEDAlarm)
{
    
    Checksum = (~(ID + AX_LEDALARM_LENGTH + AX_WRITE_DATA + AX_ALARM_LED + LEDAlarm))&0xFF;
    
	TRANSMIT_ON(gpio_status);
    tx_buffer[0] = AX_START;
    tx_buffer[1] = AX_START;
    tx_buffer[2] = ID;
    tx_buffer[3] = AX_LEDALARM_LENGTH;
    tx_buffer[4] = AX_WRITE_DATA;
    tx_buffer[5] = AX_ALARM_LED;
    tx_buffer[6] = LEDAlarm;
    tx_buffer[7] = Checksum;
    
	count = write(uart0_filestream, &tx_buffer, 8);
	if (count < 0)
	{
		printf("UART TX error\n");
	}
	
	usleep(TX_DELAY_TIME);
	TRANSMIT_OFF(gpio_status);

    return 0;
}

int JetsonAX12::setShutdownAlarm(unsigned char ID, unsigned char SALARM)
{
    
    Checksum = (~(ID + AX_SALARM_LENGTH + AX_ALARM_SHUTDOWN + AX_ALARM_LED + SALARM))&0xFF;
    
	TRANSMIT_ON(gpio_status);
    tx_buffer[0] = AX_START;
    tx_buffer[1] = AX_START;
    tx_buffer[2] = ID;
    tx_buffer[3] = AX_SALARM_LENGTH;
    tx_buffer[4] = AX_WRITE_DATA;
    tx_buffer[5] = AX_ALARM_SHUTDOWN;
    tx_buffer[6] = SALARM;
    tx_buffer[7] = Checksum;
    
	count = write(uart0_filestream, &tx_buffer, 8);
	if (count < 0)
	{
		printf("UART TX error\n");
	}
	
	usleep(TX_DELAY_TIME);
	TRANSMIT_OFF(gpio_status);

    return 0;
}

int JetsonAX12::setCMargin(unsigned char ID, unsigned char CWCMargin, unsigned char CCWCMargin)
{
    
    Checksum = (~(ID + AX_CM_LENGTH +AX_WRITE_DATA+ AX_CW_COMPLIANCE_MARGIN + CWCMargin + AX_CCW_COMPLIANCE_MARGIN + CCWCMargin))&0xFF;
    
	TRANSMIT_ON(gpio_status);
    tx_buffer[0] = AX_START;
    tx_buffer[1] = AX_START;
    tx_buffer[2] = ID;
    tx_buffer[3] = AX_CM_LENGTH;
    tx_buffer[4] = AX_WRITE_DATA;
    tx_buffer[5] = AX_CW_COMPLIANCE_MARGIN;
    tx_buffer[6] = CWCMargin;
    tx_buffer[7] = AX_CCW_COMPLIANCE_MARGIN;
    tx_buffer[8] = CCWCMargin;
    tx_buffer[9] = Checksum;
    
	count = write(uart0_filestream, &tx_buffer, 10);
	if (count < 0)
	{
		printf("UART TX error\n");
	}
	
	usleep(TX_DELAY_TIME);
	TRANSMIT_OFF(gpio_status);

    return 0;
}

int JetsonAX12::setCSlope(unsigned char ID, unsigned char CWCSlope, unsigned char CCWCSlope)
{
    
    Checksum = (~(ID + AX_CS_LENGTH +AX_WRITE_DATA+ AX_CW_COMPLIANCE_SLOPE + CWCSlope + AX_CCW_COMPLIANCE_SLOPE + CCWCSlope))&0xFF;
    
	TRANSMIT_ON(gpio_status);
    tx_buffer[0] = AX_START;
    tx_buffer[1] = AX_START;
    tx_buffer[2] = ID;
    tx_buffer[3] = AX_CS_LENGTH;
    tx_buffer[4] = AX_WRITE_DATA;
    tx_buffer[5] = AX_CW_COMPLIANCE_SLOPE;
    tx_buffer[6] = CWCSlope;
    tx_buffer[7] = AX_CCW_COMPLIANCE_SLOPE;
    tx_buffer[8] = CCWCSlope;
    tx_buffer[9] = Checksum;
    
	count = write(uart0_filestream, &tx_buffer, 10);
	if (count < 0)
	{
		printf("UART TX error\n");
	}
	
	usleep(TX_DELAY_TIME);
	TRANSMIT_OFF(gpio_status);

    return 0;
}

int JetsonAX12::setPunch(unsigned char ID, int Punch)
{

    char Punch_H,Punch_L;
    
    Punch_H = Punch >> 8;
    Punch_L = Punch;
    
	Checksum = (~(ID + AX_PUNCH_LENGTH + AX_WRITE_DATA + AX_PUNCH_L + Punch_L + Punch_H))&0xFF;
    
	TRANSMIT_ON(gpio_status);
    tx_buffer[0] = AX_START;
    tx_buffer[1] = AX_START;
    tx_buffer[2] = ID;
    tx_buffer[3] = AX_PUNCH_LENGTH;
    tx_buffer[4] = AX_WRITE_DATA;
    tx_buffer[5] = AX_PUNCH_L;
    tx_buffer[6] = Punch_L;
    tx_buffer[7] = Punch_H;
    tx_buffer[8] = Checksum;
    
	count = write(uart0_filestream, &tx_buffer, 9);
	if (count < 0)
	{
		printf("UART TX error\n");
	}
	
	usleep(TX_DELAY_TIME);
	TRANSMIT_OFF(gpio_status);

    return 0;
}

int JetsonAX12::moving(unsigned char ID)
{

    Checksum = (~(ID + AX_MOVING_LENGTH  + AX_READ_DATA + AX_MOVING + AX_BYTE_READ))&0xFF;
    
	TRANSMIT_ON(gpio_status);
    tx_buffer[0] = AX_START;
    tx_buffer[1] = AX_START;
    tx_buffer[2] = ID;
    tx_buffer[3] = AX_MOVING_LENGTH;
    tx_buffer[4] = AX_READ_DATA;
    tx_buffer[5] = AX_MOVING;
    tx_buffer[6] = AX_BYTE_READ;
    tx_buffer[7] = Checksum;
    
	count = write(uart0_filestream, &tx_buffer, 8);
	if (count < 0)
	{
		printf("UART TX error\n");
	}
	
	usleep(TX_DELAY_TIME);
	TRANSMIT_OFF(gpio_status);

	Moving_Byte = -1;
	Time_Counter = 0;
	while((bytesToRead() < 7) & (Time_Counter < TIME_OUT))
	{
	    Time_Counter++;
	    usleep(1000);
	}
	
	Read_Byte = 0;
	Read_Byte = read(uart0_filestream, &rx_buffer, bytesToRead() );
#if 0
    printf("STUFF(%d): ", ret);
    for(int i = 0; i < ret; i++)
        printf("%d ", rx_buffer[i]);
    printf("\n");
#endif	
	if(Read_Byte < 0)
	{
        printf("ERROR! NOTHING READ!\n");
        return -1;
    }
    else
    {
        for(int iter = 0; iter < (Read_Byte-1); iter++)
        {
            if( (rx_buffer[iter] == 255) & (rx_buffer[iter+1] == 255) )
            {
                if( (Error_Byte = rx_buffer[iter+4]) != 0 )
                {
                    printf("ERROR!\n");
                    return (Error_Byte * (-1));
                }
                    
                Moving_Byte = rx_buffer[iter+5];
                return Moving_Byte;  
            }
        }
    }
	    
    return Moving_Byte;
}

int JetsonAX12::lockRegister(unsigned char ID)
{
    
    Checksum = (~(ID + AX_LR_LENGTH + AX_WRITE_DATA + AX_LOCK + LOCK))&0xFF;

	TRANSMIT_ON(gpio_status);
    tx_buffer[0] = AX_START;
    tx_buffer[1] = AX_START;
    tx_buffer[2] = ID;
    tx_buffer[3] = AX_LR_LENGTH;
    tx_buffer[4] = AX_WRITE_DATA;
    tx_buffer[5] = AX_LOCK;
    tx_buffer[6] = LOCK;
    tx_buffer[7] = Checksum;
    
	count = write(uart0_filestream, &tx_buffer, 8);
	if (count < 0)
	{
		printf("UART TX error\n");
	}
	
	usleep(TX_DELAY_TIME);
	TRANSMIT_OFF(gpio_status);

    return 0;                 // Return the read error
}

int JetsonAX12::RWStatus(unsigned char ID)
{

    Checksum = (~(ID + AX_RWS_LENGTH  + AX_READ_DATA + AX_REGISTERED_INSTRUCTION + AX_BYTE_READ))&0xFF;
    
	TRANSMIT_ON(gpio_status);
    tx_buffer[0] = AX_START;
    tx_buffer[1] = AX_START;
    tx_buffer[2] = ID;
    tx_buffer[3] = AX_RWS_LENGTH;
    tx_buffer[4] = AX_READ_DATA;
    tx_buffer[5] = AX_REGISTERED_INSTRUCTION;
    tx_buffer[6] = AX_BYTE_READ;
    tx_buffer[7] = Checksum;
    
	count = write(uart0_filestream, &tx_buffer, 8);
	if (count < 0)
	{
		printf("UART TX error\n");
	}
	
	usleep(TX_DELAY_TIME);
	TRANSMIT_OFF(gpio_status);

	RWS_Byte = -1;
	Time_Counter = 0;
	while((bytesToRead() < 7) & (Time_Counter < TIME_OUT))
	{
	    Time_Counter++;
	    usleep(1000);
	}
	
	Read_Byte = 0;
	Read_Byte = read(uart0_filestream, &rx_buffer, bytesToRead() );
#if 0
    printf("STUFF(%d): ", ret);
    for(int i = 0; i < ret; i++)
        printf("%d ", rx_buffer[i]);
    printf("\n");
#endif	
	if(Read_Byte < 0)
	{
        printf("ERROR! NOTHING READ!\n");
        return -1;
    }
    else
    {
        for(int iter = 0; iter < (Read_Byte-1); iter++)
        {
            if( (rx_buffer[iter] == 255) & (rx_buffer[iter+1] == 255) )
            {
                if( (Error_Byte = rx_buffer[iter+4]) != 0 )
                {
                    printf("ERROR!\n");
                    return (Error_Byte * (-1));
                }
                    
                RWS_Byte = rx_buffer[iter+5];
                return RWS_Byte;  
            }
        }
    }
	    
    return RWS_Byte;
}

int JetsonAX12::readTemperature(unsigned char ID)
{

    memset(tx_buffer, 0, sizeof(tx_buffer) );
    memset(tx_buffer, 0, sizeof(rx_buffer) );
   
    Checksum = (~(ID + AX_TEM_LENGTH  + AX_READ_DATA + AX_PRESENT_TEMPERATURE + AX_BYTE_READ))&0xFF;
    
	TRANSMIT_ON(gpio_status);
    tx_buffer[0] = AX_START;
    tx_buffer[1] = AX_START;
    tx_buffer[2] = ID;
    tx_buffer[3] = AX_TEM_LENGTH;
    tx_buffer[4] = AX_READ_DATA;
    tx_buffer[5] = AX_PRESENT_TEMPERATURE;
    tx_buffer[6] = AX_BYTE_READ;
    tx_buffer[7] = Checksum;
    
	count = write(uart0_filestream, &tx_buffer, 8);
	if (count < 0)
	{
		printf("UART TX error\n");
	}
	
	usleep(TX_DELAY_TIME);
	TRANSMIT_OFF(gpio_status);

    
	Temperature_Byte = -1;
	Time_Counter = 0;
	while((bytesToRead() < 7) & (Time_Counter < TIME_OUT))
	{
	    Time_Counter++;
	    usleep(1000);
	}
	
	Read_Byte = 0;
	Read_Byte = read(uart0_filestream, &rx_buffer, bytesToRead() );
#if 0
    printf("STUFF(%d): ", ret);
    for(int i = 0; i < ret; i++)
        printf("%d ", rx_buffer[i]);
    printf("\n");
#endif	
	if(Read_Byte < 0)
	{
        printf("ERROR! NOTHING READ!\n");
        return -1;
    }
    else
    {
        for(int iter = 0; iter < (Read_Byte-1); iter++)
        {
            if( (rx_buffer[iter] == 255) & (rx_buffer[iter+1] == 255) )
            {
                if( (Error_Byte = rx_buffer[iter+4]) != 0 )
                {
                    printf("ERROR!\n");
                    return (Error_Byte * (-1));
                }
                    
                Temperature_Byte = rx_buffer[iter+6];
                return Temperature_Byte;  
            }
        }
    }
	    
    return Temperature_Byte;
}

int JetsonAX12::readVoltage(unsigned char ID)
{

    memset(tx_buffer, 0, sizeof(tx_buffer) );
    memset(tx_buffer, 0, sizeof(rx_buffer) );
   
    Checksum = (~(ID + AX_VOLT_LENGTH  + AX_READ_DATA + AX_PRESENT_VOLTAGE + AX_BYTE_READ))&0xFF;
    
	TRANSMIT_ON(gpio_status);
    tx_buffer[0] = AX_START;
    tx_buffer[1] = AX_START;
    tx_buffer[2] = ID;
    tx_buffer[3] = AX_VOLT_LENGTH;
    tx_buffer[4] = AX_READ_DATA;
    tx_buffer[5] = AX_PRESENT_VOLTAGE;
    tx_buffer[6] = AX_BYTE_READ;
    tx_buffer[7] = Checksum;
    
	count = write(uart0_filestream, &tx_buffer, 8);
	if (count < 0)
	{
		printf("UART TX error\n");
	}
	
	usleep(TX_DELAY_TIME);
	TRANSMIT_OFF(gpio_status);

    
	Voltage_Byte = -1;
	Time_Counter = 0;
	while((bytesToRead() < 7) & (Time_Counter < TIME_OUT))
	{
	    Time_Counter++;
	    usleep(1000);
	}
	
	Read_Byte = 0;
	Read_Byte = read(uart0_filestream, &rx_buffer, bytesToRead() );
#if 0
    printf("STUFF(%d): ", ret);
    for(int i = 0; i < ret; i++)
        printf("%d ", rx_buffer[i]);
    printf("\n");
#endif	
	if(Read_Byte < 0)
	{
        printf("ERROR! NOTHING READ!\n");
        return -1;
    }
    else
    {
        for(int iter = 0; iter < (Read_Byte-1); iter++)
        {
            if( (rx_buffer[iter] == 255) & (rx_buffer[iter+1] == 255) )
            {
                if( (Error_Byte = rx_buffer[iter+4]) != 0 )
                {
                    printf("ERROR!\n");
                    return (Error_Byte * (-1));
                }
                    
                Voltage_Byte = rx_buffer[iter+6];
                return Voltage_Byte;  
            }
        }
    }
	    
    return Voltage_Byte;
}

int JetsonAX12::readPosition(unsigned char ID)
{

    memset(tx_buffer, 0, sizeof(tx_buffer) );
    memset(tx_buffer, 0, sizeof(rx_buffer) );
   
    Checksum = (~(ID + AX_POS_LENGTH  + AX_READ_DATA + AX_PRESENT_POSITION_L + AX_BYTE_READ_POS))&0xFF;
    
	TRANSMIT_ON(gpio_status);
    tx_buffer[0] = AX_START;
    tx_buffer[1] = AX_START;
    tx_buffer[2] = ID;
    tx_buffer[3] = AX_POS_LENGTH;
    tx_buffer[4] = AX_READ_DATA;
    tx_buffer[5] = AX_PRESENT_POSITION_L;
    tx_buffer[6] = AX_BYTE_READ_POS;
    tx_buffer[7] = Checksum;
    
	count = write(uart0_filestream, &tx_buffer, 8);
	if (count < 0)
	{
		printf("UART TX error\n");
	}
	
	usleep(TX_DELAY_TIME);
	TRANSMIT_OFF(gpio_status);

    
	Position_Byte = -1;
	Time_Counter = 0;
	while((bytesToRead() < 7) & (Time_Counter < TIME_OUT))
	{
	    Time_Counter++;
	    usleep(1000);
	}
	
	Read_Byte = 0;
	Read_Byte = read(uart0_filestream, &rx_buffer, bytesToRead() );
#if 0
    printf("STUFF(%d): ", ret);
    for(int i = 0; i < ret; i++)
        printf("%d ", rx_buffer[i]);
    printf("\n");
#endif	
	if(Read_Byte < 0)
	{
        printf("ERROR! NOTHING READ!\n");
        return -1;
    }
    else
    {
        for(int iter = 0; iter < (Read_Byte-1); iter++)
        {
            if( (rx_buffer[iter] == 255) & (rx_buffer[iter+1] == 255) )
            {
                if( (Error_Byte = rx_buffer[iter+4]) != 0 )
                {
                    printf("ERROR!\n");
                    return (Error_Byte * (-1));
                }
                    
                Position_Byte = rx_buffer[iter+5] + (rx_buffer[iter+6] << 8);
                return Position_Byte;  
            }
        }
    }
	    
    return Position_Byte;
}

int JetsonAX12::readSpeed(unsigned char ID)
{

    memset(tx_buffer, 0, sizeof(tx_buffer) );
    memset(tx_buffer, 0, sizeof(rx_buffer) );
   
    Checksum = (~(ID + AX_POS_LENGTH  + AX_READ_DATA + AX_PRESENT_SPEED_L + AX_BYTE_READ_POS))&0xFF;
    
	TRANSMIT_ON(gpio_status);
    tx_buffer[0] = AX_START;
    tx_buffer[1] = AX_START;
    tx_buffer[2] = ID;
    tx_buffer[3] = AX_POS_LENGTH;
    tx_buffer[4] = AX_READ_DATA;
    tx_buffer[5] = AX_PRESENT_SPEED_L;
    tx_buffer[6] = AX_BYTE_READ_POS;
    tx_buffer[7] = Checksum;
    
	count = write(uart0_filestream, &tx_buffer, 8);
	if (count < 0)
	{
		printf("UART TX error\n");
	}
	
	usleep(TX_DELAY_TIME);
	TRANSMIT_OFF(gpio_status);

    
	Speed_Byte = -1;
	Time_Counter = 0;
	while((bytesToRead() < 7) & (Time_Counter < TIME_OUT))
	{
	    Time_Counter++;
	    usleep(1000);
	}
	
	Read_Byte = 0;
	Read_Byte = read(uart0_filestream, &rx_buffer, bytesToRead() );
#if 0
    printf("STUFF(%d): ", ret);
    for(int i = 0; i < ret; i++)
        printf("%d ", rx_buffer[i]);
    printf("\n");
#endif	
	if(Read_Byte < 0)
	{
        printf("ERROR! NOTHING READ!\n");
        return -1;
    }
    else
    {
        for(int iter = 0; iter < (Read_Byte-1); iter++)
        {
            if( (rx_buffer[iter] == 255) & (rx_buffer[iter+1] == 255) )
            {
                if( (Error_Byte = rx_buffer[iter+4]) != 0 )
                {
                    printf("ERROR!\n");
                    return (Error_Byte * (-1));
                }
                    
                Speed_Byte = rx_buffer[iter+5] + (rx_buffer[iter+6] << 8);
                return Speed_Byte;  
            }
        }
    }
	    
    return Speed_Byte;
}

int JetsonAX12::readLoad(unsigned char ID)
{

    memset(tx_buffer, 0, sizeof(tx_buffer) );
    memset(tx_buffer, 0, sizeof(rx_buffer) );
   
    Checksum = (~(ID + AX_POS_LENGTH  + AX_READ_DATA + AX_PRESENT_LOAD_L + AX_BYTE_READ_POS))&0xFF;
    
	TRANSMIT_ON(gpio_status);
    tx_buffer[0] = AX_START;
    tx_buffer[1] = AX_START;
    tx_buffer[2] = ID;
    tx_buffer[3] = AX_POS_LENGTH;
    tx_buffer[4] = AX_READ_DATA;
    tx_buffer[5] = AX_PRESENT_LOAD_L;
    tx_buffer[6] = AX_BYTE_READ_POS;
    tx_buffer[7] = Checksum;
    
	count = write(uart0_filestream, &tx_buffer, 8);
	if (count < 0)
	{
		printf("UART TX error\n");
	}
	
	usleep(TX_DELAY_TIME);
	TRANSMIT_OFF(gpio_status);

    
	Load_Byte = -1;
	Time_Counter = 0;
	while((bytesToRead() < 7) & (Time_Counter < TIME_OUT))
	{
	    Time_Counter++;
	    usleep(1000);
	}
	
	Read_Byte = 0;
	Read_Byte = read(uart0_filestream, &rx_buffer, bytesToRead() );
#if 0
    printf("STUFF(%d): ", ret);
    for(int i = 0; i < ret; i++)
        printf("%d ", rx_buffer[i]);
    printf("\n");
#endif	
	if(Read_Byte < 0)
	{
        printf("ERROR! NOTHING READ!\n");
        return -1;
    }
    else
    {
        for(int iter = 0; iter < (Read_Byte-1); iter++)
        {
            if( (rx_buffer[iter] == 255) & (rx_buffer[iter+1] == 255) )
            {
                if( (Error_Byte = rx_buffer[iter+4]) != 0 )
                {
                    printf("ERROR!\n");
                    return (Error_Byte * (-1));
                }
                    
                Load_Byte = rx_buffer[iter+5] + (rx_buffer[iter+6] << 8);
                return Load_Byte;  
            }
        }
    }
	    
    return Load_Byte;
}

int JetsonAX12::bytesToRead()
{
    int bytes = 0;
	ioctl(uart0_filestream, FIONREAD, &bytes);
    return bytes;
}


