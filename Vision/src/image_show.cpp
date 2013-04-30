#include "ros/ros.h"
#include <string.h>
#include "fmMsgs/serial.h"
#include "fmMsgs/joint_conf.h"
#include <opencv2/opencv.hpp>


using namespace std;
using namespace cv;

ros::Publisher pub;

string joint_vals[5];
bool enable;


int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_show");
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
	//ros::Subscriber sub = n.subscribe(subscriber_topic, 1, jointCallback);

	//pub = n.advertise<fmMsgs::serial>(publisher_topic, 1);

	ros::Rate loop_rate(update_frequency);

	while (ros::ok())
	{
	   VideoCapture cap = VideoCapture(0);
	    Mat frame;
	    do
	    {
		cap >> frame;
		imshow("frame",frame);
	    } while (waitKey(10) != 27 && ros::ok());
		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}
