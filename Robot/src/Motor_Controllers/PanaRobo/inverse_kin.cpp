#include "ros/ros.h"
#include <string.h>
#include "math.h"
#include "fmMsgs/serial.h"
#include "fmMsgs/joint_conf.h"
#include "fmMsgs/xyz_r_t.h"
#include "fmMsgs/serial.h"


using namespace std;

#define L1 0.4753958477
#define L2 0.374945692

ros::Publisher pub;

double position_in[3];
double rotation_in;
double tool_in;
double joint_vals[5];
double current_joint_vals[5];


void xyz_callback(fmMsgs::xyz_r_t msg)
{
	position_in[0] = msg.x;
	position_in[1] = msg.y;
	position_in[2] = msg.z;
	rotation_in = msg.phi;
	tool_in = msg.tool;
}

bool check_conf(double joint_conf[]){
	double q_min[4] = {-75.5*M_PI/180, -147*M_PI/180, -190*M_PI/180, -0.095};
	double q_max[4] = {66.3*M_PI/180, 140*M_PI/180, 190*M_PI/180, 0.095};

		// Print out joint limits
	/*
	ROS_INFO("q_min[0] = %f", q_min[0]);
	ROS_INFO("q_min[1] = %f", q_min[1]);
	ROS_INFO("q_min[2] = %f", q_min[2]);
	ROS_INFO("q_min[3] = %f", q_min[3]);

	ROS_INFO("q_min[0] = %f", q_max[0]);
	ROS_INFO("q_min[1] = %f", q_max[1]);
	ROS_INFO("q_min[2] = %f", q_max[2]);
	ROS_INFO("q_min[3] = %f", q_max[3]);
*/

	for(int i = 0; i < 4; i++){
		if(q_min[i] > joint_conf[i]){
			ROS_INFO("Joint %i failed", i);
			return false;
		}

		if(q_max[i] < joint_conf[i]){
			ROS_INFO("Joint %i failed", i);
			return false;
		}

	}
	return true;
}

void calculate_joints(){
	double x(position_in[0]), y(position_in[1]), z(position_in[2]), c(rotation_in + 0.96 + 3.14);

	if(x == 0)
		return;

	double joint_conf1[4];
	double joint_conf2[4];
	double wrist, height;

	double theta2 = 2 * atan(sqrt((((L1+L2)*(L1+L2))-((x*x)+(y*y)))/(((x*x)+(y*y))-((L1-L2)*(L1-L2)))));

	double phi = atan2(-y,x);

	double something = atan2(-(L2 * sin(theta2)), L1 +( L2 *  cos(theta2)));
	double theta=phi + something; // - (M_PI/2);
	theta2 += 3.710500 * M_PI / 180;
	theta = theta-(3.48981 * M_PI/180);// +  (3.710500 * M_PI / 180);
	wrist = c - theta - theta2;

	height = z - 0.216;

	joint_conf1[0] = theta;
	joint_conf1[1] = theta2;
	joint_conf1[2] = wrist;
	joint_conf1[3] = height;

	theta2 = -2 * atan(sqrt((((L1+L2)*(L1+L2))-((x*x)+(y*y)))/(((x*x)+(y*y))-((L1-L2)*(L1-L2)))));

	phi = atan2(-y,x);
	something = atan2(-(L2 * sin(theta2)), L1 +( L2 *  cos(theta2)));
	theta=phi + something; // - (M_PI/2);
	theta2 += 3.710500 * M_PI / 180;
	theta = theta-(3.48981 * M_PI/180);// +  (3.710500 * M_PI / 180);
	wrist = c - theta - theta2;

	joint_conf2[0] = theta;
	joint_conf2[1] = theta2;
	joint_conf2[2] = wrist;
	joint_conf2[3] = height;

	double dist1=(joint_conf1[0]-current_joint_vals[0])*(joint_conf1[0]-current_joint_vals[0])+(joint_conf1[1]-current_joint_vals[1])*(joint_conf1[1]-current_joint_vals[1])+(joint_conf1[2]-current_joint_vals[2])*(joint_conf1[2]-current_joint_vals[2]);
	double dist2=(joint_conf2[0]-current_joint_vals[0])*(joint_conf2[0]-current_joint_vals[0])+(joint_conf2[1]-current_joint_vals[1])*(joint_conf2[1]-current_joint_vals[1])+(joint_conf2[2]-current_joint_vals[2])*(joint_conf2[2]-current_joint_vals[2]);

		// Print out whether the joint confs is legal
	/*
	ROS_INFO("joint conf 1: %s",(check_conf(joint_conf1))?"true":"false");
	ROS_INFO("joint conf 2: %s",(check_conf(joint_conf2))?"true":"false");
*/
	if(check_conf(joint_conf1)){
		if (dist1<=dist2 || !check_conf(joint_conf2)){
				joint_vals[0] = joint_conf1[0];
				joint_vals[1] = joint_conf1[1];
				joint_vals[2] = joint_conf1[2];
				joint_vals[3] = joint_conf1[3];
			}
		else if(check_conf(joint_conf2)){
			joint_vals[0] = joint_conf2[0];
			joint_vals[1] = joint_conf2[1];
			joint_vals[2] = joint_conf2[2];
			joint_vals[3] = joint_conf2[3];
		}
	}
	else if(check_conf(joint_conf2)){
		joint_vals[0] = joint_conf2[0];
		joint_vals[1] = joint_conf2[1];
		joint_vals[2] = joint_conf2[2];
		joint_vals[3] = joint_conf2[3];
	}
	else{
		ROS_INFO("Cant reach the desired position");
	}

	//joint_vals[2] += 0.96;
	//joint_vals[3] = 0;
	if(tool_in == 0)
		joint_vals[4] = 15;
	else
		joint_vals[4] = 0;

		//Print the two confs
	/*
	ROS_INFO("JOINTCONF 1");
	ROS_INFO("%f", joint_conf1[0]);
	ROS_INFO("%f", joint_conf1[1]);
	ROS_INFO("%f", joint_conf1[2]);
	ROS_INFO("%f", joint_conf1[3]);

	ROS_INFO("JOINTCONF 2");
	ROS_INFO("%f", joint_conf2[0]);
	ROS_INFO("%f", joint_conf2[1]);
	ROS_INFO("%f", joint_conf2[2]);
	ROS_INFO("%f", joint_conf2[3]);
*/
	ROS_INFO("CHOSEN");
	ROS_INFO("%f", joint_vals[0]);
	ROS_INFO("%f", joint_vals[1]);
	ROS_INFO("-----");

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pana_robo_inv_kin");
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
	ros::Subscriber sub = n.subscribe(subscriber_topic, 1, xyz_callback);

	pub = n.advertise<fmMsgs::joint_conf>(publisher_topic, 1);

	ros::Rate loop_rate(update_frequency);

	position_in[0] = 0.5;
	position_in[1] = -0.3;
	position_in[2] = 0.2;
	rotation_in = 0;
	tool_in = 0;

	while (ros::ok())
	{
		calculate_joints();

		current_joint_vals[0] = joint_vals[0];
		current_joint_vals[1] = joint_vals[1];
		current_joint_vals[2] = joint_vals[2];
		current_joint_vals[3] = joint_vals[3];
		current_joint_vals[4] = joint_vals[4];
		fmMsgs::joint_conf msg;

		msg.joint1 = joint_vals[0] * 2504174.7453331477;

		msg.joint2 = (joint_vals[1] * 1669866.4839509) - 1329087;

		msg.joint3 = joint_vals[2] * 21 * (pow(2, 17)/ (2 * M_PI));

		msg.joint4 = joint_vals[3] * pow(10,7);

		msg.tool = joint_vals[4];

		msg.header.stamp = ros::Time::now();

		pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}
