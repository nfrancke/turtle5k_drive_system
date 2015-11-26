// includes
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#define ROS_FRAME_RATE 100

//create class motordrive, functions and variables
class MotorDriver123
{
public:
  double dVelAxisX;
  double dVelAxisY;
  double dVelAxisZ;

  MotorDriver123();
  void sendValue();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  ros::NodeHandle nh_;

  int iAxisX, iAxisY, iAxisZ;
  double dScaleAxisX, dScaleAxisY, dScaleAxisZ;

  ros::Publisher pub_VelPub;
  ros::Subscriber sub_VelSub;
  geometry_msgs::Twist msg;
  
};

// receive output Joystick from parameter file and set parameters
MotorDriver123::MotorDriver123(){

	// Read joy topic
	  sub_VelSub = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &MotorDriver123::joyCallback, this);
	// Publish Twist message with motorspeed
	  pub_VelPub = nh_.advertise<geometry_msgs::Twist>("mcWheelVelocityTwist", 1);

	//Check params x-axis, y-axis, z-axis
		if (nh_.hasParam("iAxisX"),nh_.hasParam("iAxisY"), nh_.hasParam("iAxisZ"))
		{
			nh_.getParam("iAxisX",iAxisX);
			nh_.getParam("iAxisY",iAxisY);
			nh_.getParam("iAxisZ",iAxisZ);
			ROS_INFO_ONCE("The motorparameters are initialized with values: Motor1: %i, Motor2: %i, Motor3: %i.", iAxisX, iAxisY, iAxisZ);
		}
		else
		{
			ROS_ERROR_ONCE("The motorparameter(s) cannot be initialized");
			iAxisX=0; iAxisY=0, iAxisZ=0;
		}

	//Check params scaler x, y, z
		if (nh_.hasParam("dScaleAxisX"),nh_.hasParam("dScaleAxisY"),nh_.hasParam("dScaleAxisZ"))
		{
			nh_.getParam("dScaleAxisX",dScaleAxisX);
			nh_.getParam("dScaleAxisY",dScaleAxisY);
			nh_.getParam("dScaleAxisZ",dScaleAxisZ);
			ROS_INFO_ONCE("The scalerparameters are initialized with values: Scaler1: %f, Scaler2: %f, Scaler3: %f.", dScaleAxisX, dScaleAxisY, dScaleAxisZ);
		}
		else
		{
			ROS_ERROR_ONCE("The scalerparameter(s) cannot be initialized");
			dScaleAxisX=0; dScaleAxisY=0, dScaleAxisZ=0;
		}
}
// set speed values and put them into a vector
void MotorDriver123::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	// calculate speedvalues
	dVelAxisX = dScaleAxisX*joy->axes[iAxisX];
	dVelAxisY = dScaleAxisY*joy->axes[iAxisY];
	dVelAxisZ = dScaleAxisZ*joy->axes[iAxisZ];
}

void MotorDriver123::sendValue(){
	//put speedvalues into vector

	msg.linear.x = dVelAxisX;
	msg.linear.y = dVelAxisY;
	msg.angular.z = dVelAxisZ;

	pub_VelPub.publish(msg);

	ROS_DEBUG("Actual motor set state: Motor1: %f, Motor2: %f, Motor3: %f.", dVelAxisX, dVelAxisY, dVelAxisZ);
	ROS_DEBUG("Actual scaler values: Scaler1: %f, Scaler2: %f, Scaler3: %f.", dScaleAxisX, dScaleAxisY, dScaleAxisZ);
}


// Main
int main(int argc, char** argv){

	ros::init(argc, argv, "MotorDrivers");
  	MotorDriver123 MotorDrivers;
	ros::Rate loop_rate(ROS_FRAME_RATE);

	while(ros::ok())
	{
		MotorDrivers.sendValue();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}


