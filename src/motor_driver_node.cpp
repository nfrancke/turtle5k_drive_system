/*****************************************************************************************************************************************
Function: 
This program responds on the receiving of a Float32MultiArray message on topic written in define variable "TOPIC_NAME.
The only data that this program will read is data[4], data[6] and data[7]. This is because serial port 5,7,8 will be written.
When the program can't open a serial port the program will be shut down. 
For debugging it is recommend to set the "log level" of ROS to -> DEBUG. normally it is setted to INFO.

Pre:
A "ros param" that descibes the factor between the incomming array (RPM) and the sending velocity to the motordrivers.
two "ros params" that describe the minimum and the maximum of the velocity that will be send to the motordrivers. NOTE: this is no RPM!!!

Post:
rs422 message on serial bus 5, 6 and 7 that gives the speed in pulses per time sample. The exact message can be found in the outputbuffer.

Writer 		: Niek Francke
date 		: 13-11-2015
********************************************************************************************************************************************/

#include <SerialStream.h>
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <string>
#include <std_msgs/Float32MultiArray.h>
#include "ros/ros.h"
#include <termios.h>
#include <fcntl.h>
#include <errno.h>

//-----settings
#define MOTORDRIVER_START_OF_FRAME 	0x5a
#define MOTORDRIVER_TYPE 			0xaa
#define MOTORDRIVER_CMD				0x03
#define MOTORDRIVER_END_OF_FRAME	0x00
#define TOPIC_NAME 					"mcWheelVelocityMps"
#define TOPIC_BUFFER_SIZE			1

using namespace std;
using namespace LibSerial;

//Define serial ports
SerialStream serial_port5;
SerialStream serial_port7;
SerialStream serial_port8;

int set_interface_attribs (int fd, int speed, int parity);
void set_blocking (int fd, int should_block);

/*****************************************************************************************************************************************
Start defining class Subscribe
********************************************************************************************************************************************/
class Subscribe
{
public:
	struct Output{ char cOutBuf[8]; int iSpeed;};	//data for serialports
	Output serialPorts[10];							
	int iConvertFactor;								//this factor will convert data from RPM to pulses/time value
	int iMaxPulseSpeed; 
	int iMinPulseSpeed;
	
	///////////////////////////////////////////////////////////////////////////////////////////
	//Function: create class and read params
	//pre: 	-
	//post: iConvertFactor will be 0 if there is no param readed.
	///////////////////////////////////////////////////////////////////////////////////////////
	Subscribe(ros::NodeHandle nh)
	{
		sub = nh.subscribe(TOPIC_NAME,TOPIC_BUFFER_SIZE, &Subscribe::commandRpmReceived, this);
		
		//Reading param. This param will be used to convert the data from RPM to pulses.
		std::string sParamName = "iConvertFactor";
		//check if param is defined.
		if (nh.hasParam(sParamName)){
			nh.getParam(sParamName, iConvertFactor);
			ROS_INFO("parameter iConvertFacotr is readed and has value :%i", iConvertFactor);
		}else{
			ROS_ERROR("Parameter iConvertFactor does not exist");
			iConvertFactor = 0;
		}

		//Reading param. This param will be used to convert the data from RPM to pulses.
		sParamName = "iMaxPulseSpeed";
		//check if param is defined.
		if (nh.hasParam(sParamName)){
			nh.getParam(sParamName, iMaxPulseSpeed);
			ROS_INFO("parameter iMaxPulseSpeed is readed and has value :%i", iMaxPulseSpeed);
		}else{
			ROS_ERROR("Parameter iMaxPulseSpeed does not exist");
			iMaxPulseSpeed = 0;
		}

		//Reading param. This param will be used to convert the data from RPM to pulses.
		sParamName = "iMinPulseSpeed";
		//check if param is defined.
		if (nh.hasParam(sParamName)){
			nh.getParam(sParamName, iMinPulseSpeed);
			ROS_INFO("parameter iMinPulseSpeed is readed and has value :%i", iMinPulseSpeed);
		}else{
			ROS_ERROR("Parameter iMinPulseSpeed does not exist");
			iMinPulseSpeed = 0;
		}
	}
	
	/////////////////////////////////////////////////////////////////////////////
	//function: this function responds on a float32MultiArray message.
	//pre: values for array places 4,6,7, because the serial ports are 5,7 and 8
	//post: -
	/////////////////////////////////////////////////////////////////////////////
	void commandRpmReceived(const std_msgs::Float32MultiArray::ConstPtr& msg)
	{
		ROS_INFO_ONCE("first time a FLoat32MultiArray is received");
		ROS_DEBUG("Float32MultiArray received");

		//prints the data from FLoat32MultiArray to log
		for(int i = 0; i < 10 ; i++){
			ROS_DEBUG("data poort %i = %f" , i, msg->data[i]);
		}

		//writes the data from Float32MultiArray to iSpeed for serialport.
		for(int i = 0 ; i < 10 ; i++){
			serialPorts[i].iSpeed = (int)msg->data[i];

			//overload protection
			if(serialPorts[i].iSpeed >= iMaxPulseSpeed){
				serialPorts[i].iSpeed = iMaxPulseSpeed;
				ROS_DEBUG("Speed is above maximum speed");
			}
			if(serialPorts[i].iSpeed <= iMinPulseSpeed){
				serialPorts[i].iSpeed = iMinPulseSpeed;
				ROS_DEBUG("Speed is below minimum speed");
			}
		}	
		
		//Make the messages for RS422 (motordrivers)	
		for(int i = 0 ; i < 10 ; i++){
			serialPorts[i].cOutBuf[0] = MOTORDRIVER_START_OF_FRAME;
			serialPorts[i].cOutBuf[1] = MOTORDRIVER_TYPE; 
			serialPorts[i].cOutBuf[2] = MOTORDRIVER_CMD; 
			serialPorts[i].cOutBuf[3] = (serialPorts[i].iSpeed & 0xff);
			serialPorts[i].cOutBuf[4] = (serialPorts[i].iSpeed >> 8) & 0xff;
			serialPorts[i].cOutBuf[5] = 0x00; //data 2 (not defined)
			serialPorts[i].cOutBuf[6] = 0x00; //data 2 (not defined)
			serialPorts[i].cOutBuf[7] = MOTORDRIVER_END_OF_FRAME;
		}

		//print speed data that will be send to motordrivers.
		for(int i = 0 ; i < 10 ; i++){
			//int iVelocity = (serialPorts[i].cOutBuf[4] << 8) + serialPorts[i].cOutBuf[3];
			ROS_DEBUG("data poort %i byte 3 = %i" , i,serialPorts[i].iSpeed);
			ROS_DEBUG("data poort %i byte 3 = %X" , i,serialPorts[i].cOutBuf[3]);
			ROS_DEBUG("data poort %i byte 4 = %X" , i,serialPorts[i].cOutBuf[4]);	
		}

		//Write serial ports. the value will be minus 1 because it will match with the serialPorts array.	
      	serial_port5.write(serialPorts[5-1].cOutBuf, 8);
      	serial_port7.write(serialPorts[7-1].cOutBuf, 8);
	   	serial_port8.write(serialPorts[8-1].cOutBuf, 8);
	}

private:
	ros::Subscriber sub;	//define ros subscriber
};
/*****************************************************************************************************************************************
end of defining class Subscribe
********************************************************************************************************************************************/

///////////////////////////////////////////////////////////////////////////////////////////
//function: initialize serial port "string sSerialPort" as follows:
//			baud = 115200, char_size = 8, parity = none, stop bits = 2 and no flauw control
//pre: 
//post: return 1 if its done. return 0 if it fails.		
////////////////////////////////////////////////////////////////////////////////////////////
bool initSerialPort(SerialStream& serial_port, string sSerialPort);

/*****************************************************************************************************************************************
Start of main
********************************************************************************************************************************************/
int main(int argc, char **argv  )
{
	//initialize ROS
	ros::init(argc, argv, "mcMotorDriver");
	
	//create nodehandle
	ros::NodeHandle nh;

	//check if init serial went good. otherwise close program.
	if((initSerialPort(serial_port5,"/dev/ttyS5") 
		&& initSerialPort(serial_port7,"/dev/ttyS7") 
		&& initSerialPort(serial_port8,"/dev/ttyS8")) 
		!= 1){
	return 1;
	}
	ROS_INFO("All serial ports are initialized");	

	//create class
	Subscribe Sobject(nh);

	//wait until a Float32MulitArray is received and run the callback function
	ros::spin();

	return 0;
}

/*****************************************************************************************************************************************
End of main
********************************************************************************************************************************************/


/*****************************************************************************************************************************************
Functions
********************************************************************************************************************************************/
//////////////////////////////////////////////////////////////////////////////////////////////
//function: initialize serial port "string sSerialPort" as follows:
//			baud = 115200, char_size = 8, parity = none, stop bits = 2 and no flauw control
//pre: 
//post: return 1 if its done. return 0 if it fails.		
//////////////////////////////////////////////////////////////////////////////////////////////
bool initSerialPort(SerialStream& serialPort,string sSerialPort){
	//
	//Open the serial port
	//
	ROS_DEBUG("Serial Port will be opened");
	//	serialPort.Open("/dev/ttyS8") ;
	serialPort.Open(sSerialPort);
	//check if serial port opens correctly
	if ( ! serialPort.good() )
    {
		ROS_ERROR("Error: Could not open serial port.");
		return 0;
	}

	ROS_INFO("Serial port is opened");

	//
	// Set the baud rate of the serial port.
	//
	serialPort.SetBaudRate( SerialStreamBuf::BAUD_115200 ) ;
	if ( ! serialPort.good() )
	{
		ROS_ERROR("Error: Could not set the baud rate.");
		return 0;
	}
	ROS_INFO("baud rate is set");

	//
	// Set the number of data bits.
	//
	serialPort.SetCharSize( SerialStreamBuf::CHAR_SIZE_8 ) ;
	if ( ! serialPort.good() )
	{
		ROS_ERROR("Error: Could not set the character size.");
		return 0;
	}
	ROS_INFO("char size is set");

	//
	// Disable parity.
	//
	serialPort.SetParity( SerialStreamBuf::PARITY_NONE ) ;
	if ( ! serialPort.good() )
	{
		ROS_ERROR("Error: Could not disable the parity.");
		return 0;
	}
	ROS_INFO("parity is setted");

	//
	// Set the number of stop bits.
	//
	serialPort.SetNumOfStopBits( 2 ) ;
	if ( ! serialPort.good() )
	{
		ROS_ERROR("Error: Could not set the number of stop bits.");
		return 0;
	}
	ROS_INFO("stop bits are set");

	//
	// Turn off hardware flow control.
	//
	serialPort.SetFlowControl( SerialStreamBuf::FLOW_CONTROL_NONE ) ;
	if ( ! serialPort.good() )
	{
		ROS_ERROR("Error: Could not use hardware flow control.");
		return 0;
	} 
	ROS_INFO("Disabled flow control");

return 1;
}

int set_interface_attribs (int fd, int speed, int parity)
{
	struct termios tty;
	memset (&tty, 0, sizeof tty);

	if (tcgetattr (fd, &tty) != 0)
	{
		ROS_ERROR("error %d from tcgetattr", errno);
		return -1;
	}

	cfsetospeed (&tty, speed);
	cfsetispeed (&tty, speed);
	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK; // disable break processing
	tty.c_lflag = 0; // no signaling chars, no echo,
	// no canonical processing
	tty.c_oflag = 0; // no remapping, no delays
	tty.c_cc[VMIN] = 0; // read doesn't block
	tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
	// enable reading
	tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
	tty.c_cflag |= parity;
	//tty.c_cflag &= ~CSTOPB;
	tty.c_cflag |= CSTOPB; //2 stop bits
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
	{
		ROS_ERROR ("error %d from tcsetattr", errno);
		return 1;
	}
	return 0;
}

void set_blocking (int fd, int should_block)
{
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	
	if (tcgetattr (fd, &tty) != 0)
	{
		ROS_ERROR ("error %d from tggetattr", errno);
	}

	tty.c_cc[VMIN] = should_block ? 1 : 0;
	tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout

	if (tcsetattr (fd, TCSANOW, &tty) != 0){
		ROS_ERROR ("error %d setting term attributes", errno);
	}
}
/*****************************************************************************************************************************************
End of functions
********************************************************************************************************************************************/
