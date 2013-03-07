#include "ros/ros.h"
#include <string.h>
#include "fmMsgs/serial.h"


using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pana_robo_joint_control");
	string publisher_topic;
	string subscriber_topic;
	int update_frequency;

		// Nodehandlers
	ros::NodeHandle n;
	ros::NodeHandle nh("~");

  	/* read parameters from ros parameter server if available otherwise use default values */
		 //Specify the publisher name
 	 nh.param<std::string> ("publisher_topic", publisher_topic, "robo_S0_tx_msg");

		//Specify the subscriber name
 	 nh.param<std::string> ("subscriber_topic", subscriber_topic, "/"); 

		//Specify the update frequency
	 nh.param<int> ("update_frequency", update_frequency, 50); 


		//Uncomment for making a callback function for the subscribed topic
	//ros::Subscriber sub = n.subscribe(subscriber_topic, 1, magCallback);

	ros::Publisher pub = n.advertise<fmMsgs::serial>(publisher_topic, 1);

	ros::Rate loop_rate(update_frequency);

	while (ros::ok())
	{		
		fmMsgs::serial msg;

		msg.data = "#W:0004 5000FF05 \n";
		msg.data += "#W:0005 FFFCCF3B \n";
		msg.data += "#W:0006 F0F0018D \n";
		msg.data += "#W:0007 001BBE44 \n";
		msg.data += "#W:000C FFFFB8FA \n";
		msg.data += "#W:000D 0017CDC0 \n";
		msg.data += "#W:000E 0017CDC0 \n";
		msg.data += "#W:000F 0017CDC0 \n";
		msg.data += "#W:0014 0017CDC0 \n";
		msg.data += "#W:0015 0000000F \n";
		msg.data += "#W:0400 5000FF05 \n";
		pub.publish(msg);

		ros::spinOnce();
	}

	return 0;
}
