#include "ros/ros.h"
#include <string.h>
#include "fmMsgs/serial.h"
#include "fmMsgs/joint_conf.h"


using namespace std;

ros::Publisher pub;

vector<string> last_joint_vals;
string joint_vals[5];
bool changed = true;
bool enable;

std::string HexToStr(const int value, int width)
{
	std::string hexStr;
	std::stringstream num;
	num.width(width);
	num.fill('0');
	num << std::fixed << std::hex << value;
	hexStr = num.str();
	return hexStr;
}

void jointCallback(fmMsgs::joint_conf jointMsg)
{
	joint_vals[0] = HexToStr(jointMsg.joint1,8);
	joint_vals[1] = HexToStr(jointMsg.joint2,8);
	joint_vals[2] = HexToStr(jointMsg.joint3,8);
	joint_vals[3] = HexToStr(jointMsg.joint4,8);
	joint_vals[4] = HexToStr(jointMsg.tool,8);

	for(int i = 0; i < 5; i++){
		if(joint_vals[i].compare(last_joint_vals[i]) != 0)
			changed = true;
		last_joint_vals[i] = joint_vals[i];
	}

	enable = jointMsg.enable;

	/*
	ROS_INFO(joint_vals[0].c_str());
	ROS_INFO(joint_vals[1].c_str());
	ROS_INFO(joint_vals[2].c_str());
	ROS_INFO(joint_vals[3].c_str());
	*/
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "joint_controller");
	string publisher_topic;
	string subscriber_topic;
	int update_frequency;
	int speed;
		// Nodehandlers
	ros::NodeHandle n;
	ros::NodeHandle nh("~");

	/* read parameters from ros parameter server if available otherwise use default values */
		//Specify the publisher name
	nh.param<std::string> ("publisher_topic", publisher_topic, "robo_S0_tx_msg");

		//Specify the subscriber name
	nh.param<std::string> ("subscriber_topic", subscriber_topic, "position");

		//Specify the update frequency
	nh.param<int> ("update_frequency", update_frequency, 50);

		// Specify at how many percent velocity to run robot
	nh.param<int> ("mst_vel", speed, 100);


		//Uncomment for making a callback function for the subscribed topic
	ros::Subscriber sub = n.subscribe(subscriber_topic, 1, jointCallback);

	pub = n.advertise<fmMsgs::serial>(publisher_topic, 1);

	ros::Rate loop_rate(update_frequency);

	//setup();

	ros::spinOnce();

	loop_rate.sleep();

	string mst_vel = HexToStr(speed, 2);

	string last_joint_vals1[5];

	for(int i = 0; i < 5; i++)
		last_joint_vals.push_back(HexToStr(0,8));



	while (ros::ok())
	{
		if(changed){

			ROS_INFO("SENDING NEW CONF");

			fmMsgs::serial msg;

			msg.data = "#w:0004 0000" + mst_vel + "05\n";

			msg.data += "#w:0005 " + joint_vals[0] + "\n";

			msg.data += "#w:0006 " + joint_vals[1] + "\n";

			msg.data += "#w:0007 " + joint_vals[2] + "\n";

			msg.data += "#w:000C " + joint_vals[3] + "\n";

			msg.data += "#w:000D 0017CDC0\n";

			msg.data += "#w:000E 0017CDC0\n";

			msg.data += "#w:000F 0017CDC0\n";

			msg.data += "#w:0014 0017CDC0\n";

			msg.data += "#w:0015 0000000F\n";

			msg.data += "#w:00C6 " + joint_vals[4] + "\n";

			msg.data += "#w:0400 00000000\n";

			pub.publish(msg);

			changed = false;
		}

		fmMsgs::serial msg_r;

		msg_r.data = "#rm:0A000000010002000300080009000A000B00100011\n";

		pub.publish(msg_r);

		ros::spinOnce();

		loop_rate.sleep();

	}

	return 0;
}
