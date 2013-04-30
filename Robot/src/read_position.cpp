#include "ros/ros.h"
#include <string.h>
#include "math.h"
#include "fmMsgs/serial.h"
#include "fmMsgs/positions.h"
#include "fmMsgs/joint_conf.h"

using namespace std;

ros::Publisher pub;

double cur_positions[4];
double desired_pos[4];

signed int hex_to_s_int(string hex){
    unsigned int x;
    std::stringstream ss;
    ss << std::hex << hex;
    ss >> x;
    // output it as a signed type
    return x;
}

double calc_dist(){
	double dist = pow(cur_positions[0]-desired_pos[0],2) + pow(cur_positions[1]-desired_pos[1],2) + pow(cur_positions[2]-desired_pos[2],2) + pow(cur_positions[3]-desired_pos[3],2);
	return sqrt(dist);
}


void ser_pos_callback(fmMsgs::serial msg)
{
	if(msg.data.substr(0,5) == "#S_RM"){
		cur_positions[0] = hex_to_s_int(msg.data.substr(14, 8));
		cur_positions[1] = hex_to_s_int(msg.data.substr(22, 8));
		cur_positions[2] = hex_to_s_int(msg.data.substr(30, 8));
		cur_positions[3] = hex_to_s_int(msg.data.substr(38, 8));
	}
}

void des_pos_callback(fmMsgs::joint_conf msg){
	desired_pos[0] = msg.joint1;
	desired_pos[1] = msg.joint2;
	desired_pos[2] = msg.joint3;
	desired_pos[3] = msg.joint4;
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "read_position");
	string publisher_topic;
	string subscriber_topic;
	string subscriber_topic_des_pos;
	int update_frequency;

		// Nodehandlers
	ros::NodeHandle n;
	ros::NodeHandle nh("~");

	/* read parameters from ros parameter server if available otherwise use default values */
		//Specify the publisher name
	nh.param<std::string> ("publisher_topic", publisher_topic, "/Robot/Positions");

		//Specify the subscriber name
	nh.param<std::string> ("subscriber_topic_pos", subscriber_topic, "/Serials/robo_S0_rx_msg");

	nh.param<std::string> ("subscriber_topic_des_pos", subscriber_topic_des_pos, "/Robot/joint_configuration");


		//Specify the update frequency
	nh.param<int> ("update_frequency", update_frequency, 100);

		//Uncomment for making a callback function for the subscribed topic
	ros::Subscriber sub = n.subscribe(subscriber_topic, 1, ser_pos_callback);
	ros::Subscriber sub1 = n.subscribe(subscriber_topic_des_pos, 1, des_pos_callback);

	pub = n.advertise<fmMsgs::positions>(publisher_topic, 1);

	ros::Rate loop_rate(update_frequency);

	while (ros::ok())
	{
		fmMsgs::positions msg;

		/*
		ROS_INFO("-------------------------");

		ROS_INFO("CURRENT:");

		ROS_INFO("join_pos_1: %f", cur_positions[0]);
		ROS_INFO("join_pos_2: %f", cur_positions[1]);
		ROS_INFO("join_pos_3: %f", cur_positions[2]);
		ROS_INFO("join_pos_4: %f", cur_positions[3]);

		ROS_INFO("DESIRED:");

		ROS_INFO("join_pos_1: %f", desired_pos[0]);
		ROS_INFO("join_pos_2: %f", desired_pos[1]);
		ROS_INFO("join_pos_3: %f", desired_pos[2]);
		ROS_INFO("join_pos_4: %f", desired_pos[3]);
		*/

		msg.joint1 = cur_positions[0];
		msg.joint2 = cur_positions[1];
		msg.joint3 = cur_positions[2];
		msg.joint4 = cur_positions[3];
		msg.distance = calc_dist();

		msg.header.stamp = ros::Time::now();


		pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();

	}

	return 0;
}
