#include "ros/ros.h"
#include <string.h>
#include "fmMsgs/xyz_r_t.h"


using namespace std;

ros::Publisher pub;

vector<double> positions;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "lego_test");
	string publisher_topic;
	string subscriber_topic;
	int update_frequency;

		// Nodehandlers
	ros::NodeHandle n;
	ros::NodeHandle nh("~");

	/* read parameters from ros parameter server if available otherwise use default values */
		//Specify the publisher name
	nh.param<std::string> ("publisher_topic", publisher_topic, "/Robot/robot_xyz");

		//Specify the subscriber name
	nh.param<std::string> ("subscriber_topic", subscriber_topic, "/");

		//Specify the update frequency
	nh.param<int> ("update_frequency", update_frequency, 1);

		//Uncomment for making a callback function for the subscribed topic
	//ros::Subscriber sub = n.subscribe(subscriber_topic, 1, jointCallback);

	pub = n.advertise<fmMsgs::xyz_r_t>(publisher_topic, 1);

	ros::Rate loop_rate(update_frequency);

	double conf1[] = {0.5, -0.3, 0.2, 0, 1};
	double conf2[] = {0.5, -0.3, 0.126, 0, 1};
	double conf3[] = {0.5, -0.3, 0.126, 0, 0};
	double conf4[] = {0.5, -0.3, 0.2, 0, 0};
	double conf5[] = {0.43, -0.45, 0.15, -1.57, 0};
	double conf6[] = {0.43, -0.45, 0.15, -1.57, 1};
	double conf7[] = {0.5, -0.3, 0.2, 0, 1};


	int turn = -1;

	while (ros::ok())
	{		
		fmMsgs::xyz_r_t msg;

		if(turn == -1 || turn == 0){
			msg.x = conf1[0];
			msg.y = conf1[1];
			msg.z = conf1[2];
			msg.phi = conf1[3];
			msg.tool = conf1[4];
			turn++;
		}
		else if(turn == 1){
			msg.x = conf2[0];
			msg.y = conf2[1];
			msg.z = conf2[2];
			msg.phi = conf2[3];
			msg.tool = conf2[4];
			turn = 2;
		}
		else if(turn == 2){
			msg.x = conf3[0];
			msg.y = conf3[1];
			msg.z = conf3[2];
			msg.phi = conf3[3];
			msg.tool = conf3[4];
			turn = 3;
		}
		else if(turn == 3){
			msg.x = conf4[0];
			msg.y = conf4[1];
			msg.z = conf4[2];
			msg.phi = conf4[3];
			msg.tool = conf4[4];
			turn = 4;
		}
		else if(turn == 4){
			msg.x = conf5[0];
			msg.y = conf5[1];
			msg.z = conf5[2];
			msg.phi = conf5[3];
			msg.tool = conf5[4];
			turn = 5;
		}
		else if(turn == 5){
			msg.x = conf6[0];
			msg.y = conf6[1];
			msg.z = conf6[2];
			msg.phi = conf6[3];
			msg.tool = conf6[4];
			turn = 6;
		}
		else if(turn == 6){
			msg.x = conf7[0];
			msg.y = conf7[1];
			msg.z = conf7[2];
			msg.phi = conf7[3];
			msg.tool = conf7[4];
			turn = 6;
		}

		pub.publish(msg);

		ros::spinOnce();

		//loop_rate.sleep();
		//loop_rate.sleep();
		loop_rate.sleep();
	}

	return 0;
}
