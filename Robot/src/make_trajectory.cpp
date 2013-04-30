#include "ros/ros.h"
#include <string.h>
#include "math.h"
#include "fmMsgs/xyz_r_t.h"
#include "fmMsgs/serial.h"
#include "fmMsgs/joint_conf.h"
#include "fmMsgs/positions.h"

using namespace std;

ros::Publisher pub;

vector< vector<double> > positions;
vector< vector<double> > trajectory;

signed int cur_positions[4];
signed int desired_pos[4];
double MAX_DIST = 10;

vector<double> safe_position; // = {0.4, -0.4, 0.2, 0, 1};
vector<double> second_pipe;

signed int hex_to_s_int(string hex){
    unsigned int x;
    std::stringstream ss;
    ss << std::hex << hex;
    ss >> x;
    // output it as a signed type
    return x;
}

void ser_pos_callback(fmMsgs::positions msg)
{
	cur_positions[0] = msg.joint1;
	cur_positions[1] = msg.joint2;
	cur_positions[2] = msg.joint3;
	cur_positions[3] = msg.joint4;
}

void des_pos_callback(fmMsgs::joint_conf msg){
	desired_pos[0] = msg.joint1;
	desired_pos[1] = msg.joint2;
	desired_pos[2] = msg.joint3;
	desired_pos[3] = msg.joint4;
}

void calc_trajectory(){
	for(int i = 0; i < positions.size(); i++){
		// Move over the brick
		vector<double> temp_pos;
		temp_pos.push_back(positions.at(i).at(0));
		temp_pos.push_back(positions.at(i).at(1));
		temp_pos.push_back(safe_position.at(2));
		temp_pos.push_back(positions.at(i).at(3));
		temp_pos.push_back(1);
		trajectory.push_back(temp_pos);


		// Move down to the brick
		vector<double> temp_pos1;
		temp_pos1.push_back(positions.at(i).at(0));
		temp_pos1.push_back(positions.at(i).at(1));
		temp_pos1.push_back(positions.at(i).at(2));
		temp_pos1.push_back(positions.at(i).at(3));
		temp_pos1.push_back(1);
		trajectory.push_back(temp_pos1);


		//Grab the brick
		vector<double> temp_pos2;
		temp_pos2.push_back(positions.at(i).at(0));
		temp_pos2.push_back(positions.at(i).at(1));
		temp_pos2.push_back(positions.at(i).at(2));
		temp_pos2.push_back(positions.at(i).at(3));
		temp_pos2.push_back(0);
		trajectory.push_back(temp_pos2);

		// Lift the brick
		vector<double> temp_pos3;
		temp_pos3.push_back(positions.at(i).at(0));
		temp_pos3.push_back(positions.at(i).at(1));
		temp_pos3.push_back(safe_position[2]);
		temp_pos3.push_back(positions.at(i).at(3));
		temp_pos3.push_back(0);
		trajectory.push_back(temp_pos3);

		//Move to second pipe
		vector<double> temp_pos4;
		temp_pos4.push_back(second_pipe.at(0));
		temp_pos4.push_back(second_pipe.at(1));
		temp_pos4.push_back(safe_position[2]);
		temp_pos4.push_back(second_pipe.at(3));
		temp_pos4.push_back(0);
		trajectory.push_back(temp_pos4);

		//Move down to the pipe
		vector<double> temp_pos5;
		temp_pos5.push_back(second_pipe.at(0));
		temp_pos5.push_back(second_pipe.at(1));
		temp_pos5.push_back(second_pipe.at(2));
		temp_pos5.push_back(second_pipe.at(3));
		temp_pos5.push_back(0);
		trajectory.push_back(temp_pos5);

		// Release
		vector<double> temp_pos6;
		temp_pos6.push_back(second_pipe.at(0));
		temp_pos6.push_back(second_pipe.at(1));
		temp_pos6.push_back(second_pipe.at(2));
		temp_pos6.push_back(second_pipe.at(3));
		temp_pos6.push_back(1);
		trajectory.push_back(temp_pos6);

		//Move to safe position
		trajectory.push_back(safe_position);
	}
}

double calc_dist(){
	double dist = pow(cur_positions[0]-desired_pos[0],2) + pow(cur_positions[1]-desired_pos[1],2) + pow(cur_positions[2]-desired_pos[2],2) + pow(cur_positions[3]-desired_pos[3],2);
	return sqrt(dist);
}

int main(int argc, char **argv)
{
	safe_position.push_back(0.4);
	safe_position.push_back(-0.4);
	safe_position.push_back(0.16);
	safe_position.push_back(0);
	safe_position.push_back(1);

	second_pipe.push_back(0.43);
	second_pipe.push_back(-0.45);
	second_pipe.push_back(0.15);
	second_pipe.push_back(-1.57);
	second_pipe.push_back(0);


	// PUSH BACK SAMPLE POSITIONS
	vector<double> temp;
	temp.push_back(0.5);
	temp.push_back(-0.3);
	temp.push_back(0.128);
	temp.push_back(0);
	temp.push_back(0);
	positions.push_back(temp);

	vector<double> temp1;
	temp1.push_back(0.3);
	temp1.push_back(-0.36);
	temp1.push_back(0.123);
	temp1.push_back(-1.2);
	temp1.push_back(0);
	positions.push_back(temp1);

	/*
	vector<double> temp2;
	temp2.push_back(0.116);
	temp2.push_back(-0.46);
	temp2.push_back(0.123);
	temp2.push_back(-2);
	temp2.push_back(0);
	positions.push_back(temp2);
	*/

	trajectory.push_back(safe_position);

	calc_trajectory();

	ros::init(argc, argv, "make_trajectory");
	string publisher_topic;
	string subscriber_topic;
	string subscriber_topic_des_pos;
	int update_frequency;

		// Nodehandlers
	ros::NodeHandle n;
	ros::NodeHandle nh("~");

	/* read parameters from ros parameter server if available otherwise use default values */
		//Specify the publisher name
	nh.param<std::string> ("publisher_topic", publisher_topic, "/Robot/robot_xyz");

	nh.param<double> ("max_dist", MAX_DIST, 1000);

		//Specify the subscriber name
	nh.param<std::string> ("subscriber_topic_pos", subscriber_topic, "/Robot/position");

	nh.param<std::string> ("subscriber_topic_des_pos", subscriber_topic_des_pos, "/Robot/joint_configuration");


		//Specify the update frequency
	nh.param<int> ("update_frequency", update_frequency, 1);

		//Uncomment for making a callback function for the subscribed topic
	ros::Subscriber sub = n.subscribe(subscriber_topic, 1, ser_pos_callback);
	ros::Subscriber sub1 = n.subscribe(subscriber_topic_des_pos, 1, des_pos_callback);

	pub = n.advertise<fmMsgs::xyz_r_t>(publisher_topic, 1);

	ros::Rate loop_rate(update_frequency);
/*
	double conf1[] = {0.5, -0.3, 0.2, 0, 1};
	double conf2[] = {0.5, -0.3, 0.126, 0, 1};
	double conf3[] = {0.5, -0.3, 0.126, 0, 0};
	double conf4[] = {0.5, -0.3, 0.2, 0, 0};
	double conf5[] = {0.43, -0.45, 0.15, -1.57, 0};
	double conf6[] = {0.43, -0.45, 0.15, -1.57, 1};
	double conf7[] = {0.5, -0.3, 0.2, 0, 1};

*/
	desired_pos[0] = 999999;
	desired_pos[1] = 999999;
	desired_pos[2] = 999999;
	desired_pos[3] = 999999;

	vector<double> current_pos;

	while (ros::ok())
	{
		if(!trajectory.empty() && calc_dist() < 1500){
			current_pos = trajectory.front();
			trajectory.erase(trajectory.begin());

			fmMsgs::xyz_r_t msg;

			msg.x = current_pos[0];
			msg.y = current_pos[1];
			msg.z = current_pos[2];
			msg.phi = current_pos[3];
			msg.tool = current_pos[4];

			msg.header.stamp = ros::Time::now();

			pub.publish(msg);
		}

		ROS_INFO("-------------------------");

		ROS_INFO("join_pos_1: %i", cur_positions[0]);
		ROS_INFO("join_pos_2: %i", cur_positions[1]);
		ROS_INFO("join_pos_3: %i", cur_positions[2]);
		ROS_INFO("join_pos_4: %i", cur_positions[3]);

		ROS_INFO("join_pos_1: %i", desired_pos[0]);
		ROS_INFO("join_pos_2: %i", desired_pos[1]);
		ROS_INFO("join_pos_3: %i", desired_pos[2]);
		ROS_INFO("join_pos_4: %i", desired_pos[3]);

		ROS_INFO("Distance: %f", calc_dist());
		ROS_INFO("Trajectory size: %i", trajectory.size());

		ros::spinOnce();

		loop_rate.sleep();
		loop_rate.sleep();
		loop_rate.sleep();


	}

	return 0;
}
