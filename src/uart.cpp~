#include <SerialStream.h>
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <string>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>

#include "ros/ros.h"

using namespace std;
using namespace LibSerial;

//
// Init the serial port.
//
SerialStream serial_port5;
SerialStream serial_port7;
SerialStream serial_port8;

class Subscribe
{
public:
	struct Output{ char cOutBuf[8]; int iSpeed;};
	Output serialPorts[10];	

	Subscribe(ros::NodeHandle nh)
	{
		sub = nh.subscribe("mcWheelVelocityMps",1000, &Subscribe::commandRpmReceived, this);
		
//		const std::string sParamName = "iConvertFactor";
//		int iTest;
//		if(!ros::param::get(sParamName,iTest)){
//			ROS_ERROR("Could not get parameter");
//		}
	}

	void commandRpmReceived(const std_msgs::Float32MultiArray::ConstPtr& msg)
	{
		ROS_INFO("Float32MultiArray received");

		for(int i = 0; i < 10 ; i++){
			ROS_INFO("data poort %i = %f" , i, msg->data[i]);
		}

		for(int i = 0 ; i < 10 ; i++){
			serialPorts[i].iSpeed = msg->data[i];
		}	

		//for testing
//		serialPorts[5].iSpeed = msg->data[0];
//		serialPorts[7].iSpeed = msg->data[0];
//		serialPorts[8].iSpeed = msg->data[0];
				
		for(int i = 0 ; i < 10 ; i++){
			serialPorts[i].cOutBuf[0] = 0x5a; //start of frame
			serialPorts[i].cOutBuf[1] = 0xaa; //start of frame
			serialPorts[i].cOutBuf[2] = 0x03; //start of frame
			serialPorts[i].cOutBuf[3] = (serialPorts[i].iSpeed & 0xff);			//speed
			serialPorts[i].cOutBuf[4] = (serialPorts[i].iSpeed >> 8) & 0xff;	//speed
			serialPorts[i].cOutBuf[5] = 0x00; //??
			serialPorts[i].cOutBuf[6] = 0x00; //??
			serialPorts[i].cOutBuf[7] = 0x00; //eof
		}

		ROS_INFO("data poort 5 = %i" , serialPorts[5-1].cOutBuf[3]);
		ROS_INFO("data poort 5 = %i" , serialPorts[5-1].cOutBuf[4]);	
			
      	serial_port5.write(serialPorts[5-1].cOutBuf, 8);
      	serial_port7.write(serialPorts[7-1].cOutBuf, 8);
	   	serial_port8.write(serialPorts[8-1].cOutBuf, 8);
	}

private:
	ros::Subscriber sub;
};//end of class Subscribe

bool initSerialPort(SerialStream& serial_port, string sSerialPort);


int main(int argc, char **argv  )
{
	//initialize ROS
	ros::init(argc, argv, "mcMotorDriver");
	
	ros::NodeHandle nh;

	//check if init serial went good.
	if((initSerialPort(serial_port5,"/dev/ttyS5") 
		&& initSerialPort(serial_port7,"/dev/ttyS7") 
		&& initSerialPort(serial_port8,"/dev/ttyS8")) 
		!= 1){
	return 1;
	}
	ROS_INFO("All serial ports are initialized");	

	//create class
	Subscribe Sobject(nh);

	//	ros::Subscriber sub = nh.subscribe("mcWheelVelocityMps",1000, commandRpmReceived);	

	ros::spin();

	return 0;
}

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
	ROS_INFO("baud rate is setted");

	//
	// Set the number of data bits.
	//
	serialPort.SetCharSize( SerialStreamBuf::CHAR_SIZE_8 ) ;
	if ( ! serialPort.good() )
	{
		ROS_ERROR("Error: Could not set the character size.");
		return 0;
	}
	ROS_INFO("char size is setted");

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
	ROS_INFO("stop bits are setted");

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
