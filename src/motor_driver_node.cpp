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
#define PUB_TOPIC_NAME				"motorspeed_feedback"
#define PUB_TOPIC_BUFFER_SIZE		1

#define SERIAL_PORT_5				4
#define SERIAL_PORT_7				6
#define SERIAL_PORT_8				7

using namespace std;


int set_interface_attribs (int fd, int speed, int parity);
void set_blocking (int fd, int should_block);

/*****************************************************************************************************************************************
Start defining class Subscribe
********************************************************************************************************************************************/
class Subscribe
{
public:
	struct Output{ char cOutBuf[8]; int iSpeed; unsigned char cInBuf[1000];};	//data for serialports
	Output serialPorts[10];							
	int iConvertFactor;								//this factor will convert data from RPM to pulses/time value
	int iMaxPulseSpeed; 
	int iMinPulseSpeed;
	int iSerialPortId[10];							
	
	///////////////////////////////////////////////////////////////////////////////////////////
	//Function: create class and read params
	//pre: 	-
	//post: iConvertFactor will be 0 if there is no param readed.
	///////////////////////////////////////////////////////////////////////////////////////////
	Subscribe(ros::NodeHandle nh)
	{
		sub = nh.subscribe(TOPIC_NAME,TOPIC_BUFFER_SIZE, &Subscribe::commandRpmReceived, this);

		pub = nh.advertise<std_msgs::Float32MultiArray>(PUB_TOPIC_NAME, PUB_TOPIC_BUFFER_SIZE);
		
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

		//Write data.
		write(iSerialPortId[SERIAL_PORT_5], serialPorts[SERIAL_PORT_5].cOutBuf,sizeof serialPorts[SERIAL_PORT_5].cOutBuf);
		write(iSerialPortId[SERIAL_PORT_7], serialPorts[SERIAL_PORT_7].cOutBuf,sizeof serialPorts[SERIAL_PORT_7].cOutBuf);
		write(iSerialPortId[SERIAL_PORT_8], serialPorts[SERIAL_PORT_8].cOutBuf,sizeof serialPorts[SERIAL_PORT_8].cOutBuf);
	}

	bool readSerialPort(){
		//create multiarray
		std_msgs::Float32MultiArray msg;
	
		if ((read(iSerialPortId[SERIAL_PORT_5], serialPorts[SERIAL_PORT_5].cInBuf,sizeof serialPorts[SERIAL_PORT_5].cInBuf))>0){
			iSerial5NewData = true;
		}
		if ((read(iSerialPortId[SERIAL_PORT_7], serialPorts[SERIAL_PORT_7].cInBuf,sizeof serialPorts[SERIAL_PORT_7].cInBuf))>0){
			iSerial7NewData = true;
		}
		if ((read(iSerialPortId[SERIAL_PORT_8], serialPorts[SERIAL_PORT_8].cInBuf,sizeof serialPorts[SERIAL_PORT_8].cInBuf))>0){
			iSerial8NewData = true;
		} 

		if(iSerial5NewData && iSerial7NewData && iSerial8NewData){
			
			int iEncoderData;

			ROS_INFO("data encoder:%x%x", serialPorts[SERIAL_PORT_5].cInBuf[4],serialPorts[SERIAL_PORT_5].cInBuf[3]);
			ROS_INFO("data encoder:%x%x", serialPorts[SERIAL_PORT_7].cInBuf[4],serialPorts[SERIAL_PORT_7].cInBuf[3]);
			ROS_INFO("data encoder:%x%x", serialPorts[SERIAL_PORT_8].cInBuf[4],serialPorts[SERIAL_PORT_8].cInBuf[3]);

			//put speedvalues into array
			msg.data.clear();
			msg.data.push_back(0);
			msg.data.push_back(0);
			msg.data.push_back(0);
			msg.data.push_back(0);
			iEncoderData = (serialPorts[SERIAL_PORT_5].cInBuf[4] << 8) | (serialPorts[SERIAL_PORT_5].cInBuf[3]);
			msg.data.push_back(iEncoderData);
			msg.data.push_back(0);
			iEncoderData = (serialPorts[SERIAL_PORT_7].cInBuf[4] << 8) | (serialPorts[SERIAL_PORT_7].cInBuf[3]);
			msg.data.push_back(iEncoderData);
			iEncoderData = (serialPorts[SERIAL_PORT_8].cInBuf[4] << 8) | (serialPorts[SERIAL_PORT_8].cInBuf[3]);		
			msg.data.push_back(iEncoderData);
			msg.data.push_back(0);

			//send message
			pub.publish(msg);

			//clear markers
			iSerial5NewData = false;
			iSerial7NewData = false;
			iSerial8NewData = false;
			ROS_INFO("Send encoder data");
			return 1;
		}else{
			return 0;
		}
	}

private:
	ros::Subscriber sub;	//define ros subscriber
	ros::Publisher	pub;	//define ros publisher
	bool iSerial5NewData;
	bool iSerial7NewData;
	bool iSerial8NewData;
};
/*****************************************************************************************************************************************
end of defining class Subscribe
********************************************************************************************************************************************/

/*****************************************************************************************************************************************
Start of main
********************************************************************************************************************************************/
int main(int argc, char **argv  )
{
	//initialize ROS
	ros::init(argc, argv, "mcMotorDriver");
	
	//create nodehandle
	ros::NodeHandle nh;

	//create class
	Subscribe Sobject(nh);
	
	ROS_INFO("Serial Ports will be initialized");
	ROS_DEBUG("O_NONBLOCK = %i", O_NONBLOCK);

	//open the serial ports and put the id number in a variable
	//O_RWDR = open for reading and writing
	//O_NOCTTY = the port never becomes the controlling terminal of the process
	//O_NDELAY = use non-blocking i/o. on some system this is also means the rs232 dcd signal line is ignored.	
	Sobject.iSerialPortId[SERIAL_PORT_5] = open("/dev/ttyS5", O_RDWR | O_NONBLOCK);//O_RDWR | O_NOCTTY | O_SYNC);
	ROS_INFO("Serial port 5 are connected to hardware");
	Sobject.iSerialPortId[SERIAL_PORT_7] = open("/dev/ttyS7", O_RDWR | O_NONBLOCK);//O_RDWR | O_NOCTTY | O_SYNC);
	ROS_INFO("Serial port 7 are connected to hardware");
	Sobject.iSerialPortId[SERIAL_PORT_8] = open("/dev/ttyS8", O_RDWR | O_NONBLOCK);//O_RDWR | O_NOCTTY | O_SYNC);
	ROS_INFO("Serial port 8 are connected to hardware");

	//control of all ports are opened correctly
	if(Sobject.iSerialPortId[SERIAL_PORT_5] > 0){
		ROS_INFO("serial port 5 is number %i",Sobject.iSerialPortId[SERIAL_PORT_5]);
	} else {
		ROS_ERROR ("error %d opening %s: %s", errno, "/dev/ttyS5", strerror (errno));
		return -1;
	}
	if(Sobject.iSerialPortId[SERIAL_PORT_7] > 0){
		ROS_INFO("serial port 7 is number %i",Sobject.iSerialPortId[SERIAL_PORT_7]);
	}else {
		ROS_ERROR ("error %d opening %s: %s", errno, "/dev/ttyS7", strerror (errno));
		return -1;
	}
	if(Sobject.iSerialPortId[SERIAL_PORT_8] > 0){
		ROS_INFO("serial port 8 is number %i",Sobject.iSerialPortId[SERIAL_PORT_8]);
	}else {
		ROS_ERROR ("error %d opening %s: %s", errno, "/dev/ttyS8", strerror (errno));
		return -1;
	}

	//set parameters for serial communication
	set_interface_attribs (Sobject.iSerialPortId[SERIAL_PORT_5], B115200, 0); // set speed to 115,200 bps, 8n1 (no parity)
	set_interface_attribs (Sobject.iSerialPortId[SERIAL_PORT_7], B115200, 0); // set speed to 115,200 bps, 8n1 (no parity)
	set_interface_attribs (Sobject.iSerialPortId[SERIAL_PORT_8], B115200, 0); // set speed to 115,200 bps, 8n1 (no parity)
	set_blocking (Sobject.iSerialPortId[SERIAL_PORT_5], 0); // set no blocking
	set_blocking (Sobject.iSerialPortId[SERIAL_PORT_7], 0); // set no blocking
	set_blocking (Sobject.iSerialPortId[SERIAL_PORT_8], 0); // set no blocking

	ROS_INFO("Serial ports are initialized");

	while(ros::ok())
	{
		Sobject.readSerialPort();

		//wait until a Float32MulitArray is received and run the callback function
		ros::spinOnce();
//		ros::spin();
	}

	return 0;
}

/*****************************************************************************************************************************************
End of main
********************************************************************************************************************************************/


/*****************************************************************************************************************************************
Functions
********************************************************************************************************************************************/

/////////////////////////////////////////////////////////////////////////////
//function: this function set the parameters for the serial port.
//pre: 
//post: -
/////////////////////////////////////////////////////////////////////////////
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
//	tty.c_lflag = 0; // no signaling chars, no echo,
	// no canonical processing
	tty.c_lflag &= ~ICANON; // no signaling chars, no echo,
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
	tty.c_cflag &= ~CRTSCTS; //disable hardware flow control

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
	{
		ROS_ERROR ("error %d from tcsetattr", errno);
		return 1;
	}
	return 0;
}

/////////////////////////////////////////////////////////////////////////////
//function: this function sets the blocking of te serial port.
//pre: 
//post: -
/////////////////////////////////////////////////////////////////////////////
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
