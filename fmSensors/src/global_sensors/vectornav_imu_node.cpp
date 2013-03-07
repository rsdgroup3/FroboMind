/*
 * vectornav_imu_node.cpp
 *
 *  Created on: Apr 20, 2012
 *      Author: morl
 */

#include <stdio.h>
#include <string.h>
#include "ros/ros.h"
#include <fmMsgs/serial.h>
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Pose.h"
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <tf/tf.h>

typedef boost::tokenizer<boost::char_separator<char> > tokenizer;

class VectorNav
{
public:
	VectorNav()
	{


	}

	~VectorNav()
	{

	}

	void set_covariance(double cov_x, double cov_y, double cov_z)
	{
		imu_msg.orientation_covariance[0] = cov_x;
		imu_msg.orientation_covariance[4] = cov_y;
		imu_msg.orientation_covariance[8] = cov_z;
	}

	void processSerialCallback(const fmMsgs::serial::ConstPtr& rx_msg)
	{
		boost::char_separator<char> sep("$*,");
		tokenizer::iterator tok_iter;
		tokenizer tokens(rx_msg->data, sep);

		tok_iter = tokens.begin();
		// get NMEA identifier
		if((*tok_iter).compare("VNQMR") == 0)
		{
			processIMU(tokens,rx_msg->data);
		}
		else
		{
			ROS_INFO("Ignoring Unknown NMEA identifier %s",(*tok_iter).c_str());
		}

	}
	bool enu_selected;
	sensor_msgs::Imu imu_msg;
	ros::Publisher imu_pub;
	std::string frame_id;

private:

	void processIMU(tokenizer& tokens,std::string raw)
	{
		unsigned int chk,calc_chk;
		tokenizer::iterator tok_iter = tokens.begin();

		// skip identifier
		tok_iter++;

		if(countTokens(tokens) == 15)
		{
			chk = extractChecksum(tokens,14);
			calc_chk = calculateChecksum(raw);
			if(chk == calc_chk)
			{
				try
				{
					imu_msg.header.stamp = ros::Time::now();
					imu_msg.header.frame_id = frame_id;


					if(enu_selected)
					{
						// ENU ORIENTATION
						/* swap x and y and negate z
						 *
						 * */
						imu_msg.orientation.y = boost::lexical_cast<float>(*tok_iter++);
						imu_msg.orientation.x = boost::lexical_cast<float>(*tok_iter++);
						imu_msg.orientation.z = - boost::lexical_cast<float>(*tok_iter++);
						imu_msg.orientation.w = boost::lexical_cast<float>(*tok_iter++);

						tok_iter++; //skip magnetometer
						tok_iter++;
						tok_iter++;

						// acceleration swap x y negate z
						imu_msg.linear_acceleration.y = boost::lexical_cast<float>(*tok_iter++);
						imu_msg.linear_acceleration.x = boost::lexical_cast<float>(*tok_iter++);
						imu_msg.linear_acceleration.z = - boost::lexical_cast<float>(*tok_iter++);

						// angular rates swap  x y and negate z
						imu_msg.angular_velocity.y = boost::lexical_cast<float>(*tok_iter++);
						imu_msg.angular_velocity.x = boost::lexical_cast<float>(*tok_iter++);
						imu_msg.angular_velocity.z = -boost::lexical_cast<float>(*tok_iter++);
					}
					else
					{
						// NED orientation
						imu_msg.orientation.x = boost::lexical_cast<float>(*tok_iter++);
						imu_msg.orientation.y = boost::lexical_cast<float>(*tok_iter++);
						imu_msg.orientation.z = boost::lexical_cast<float>(*tok_iter++);
						imu_msg.orientation.w = boost::lexical_cast<float>(*tok_iter++);

						tok_iter++; //skip magnetometer
						tok_iter++;
						tok_iter++;

						// acceleration
						imu_msg.linear_acceleration.x = boost::lexical_cast<float>(*tok_iter++);
						imu_msg.linear_acceleration.y = boost::lexical_cast<float>(*tok_iter++);
						imu_msg.linear_acceleration.z = boost::lexical_cast<float>(*tok_iter++);

						// angular rates
						imu_msg.angular_velocity.x = boost::lexical_cast<float>(*tok_iter++);
						imu_msg.angular_velocity.y = boost::lexical_cast<float>(*tok_iter++);
						imu_msg.angular_velocity.z = boost::lexical_cast<float>(*tok_iter++);
					}

					imu_pub.publish(imu_msg);

				}
				catch(boost::bad_lexical_cast &)
				{
					ROS_ERROR("Could not convert accelerometer and gyro readings");
				}

			}
			else
			{
				ROS_WARN("Checksum did not match");
			}
		}

	}

	unsigned int extractChecksum(tokenizer tokens,int chk_msg_start)
	{
		tokenizer::iterator tok_iter_chk;
		unsigned int chk;

		tok_iter_chk = tokens.begin();

		for(int j=0;j<chk_msg_start;j++)
		{
			tok_iter_chk++;
		}

		std::stringstream ss(*tok_iter_chk);
		ss >> std::hex >> chk;

		return chk;
	}

	unsigned int calculateChecksum(std::string s)
	{
		uint8_t chk = 0;

		for(unsigned int i=1;i<s.length();i++)
		{
			if(s.at(i) == '*')
			{
				break;
			}
			chk ^= (uint8_t)s.at(i);
		}

		return chk;
	}

	unsigned int countTokens(tokenizer& tokens)
	{
		tokenizer::iterator tok_iter;
		unsigned int count = 0;
		for (tok_iter = tokens.begin(); tok_iter != tokens.end(); ++tok_iter)
		{
			count++;
		}

		return count;
	}

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_parser");
  ros::NodeHandle nh("~");
  ros::NodeHandle n;

  VectorNav imu;

  std::string subscribe_topic_id;
  std::string publish_topic_id;

  double cov_x,cov_y,cov_z;

  nh.param<std::string> ("subscribe_topic_id", subscribe_topic_id, "fmCSP/com1_rx");
  nh.param<std::string> ("publish_topic_id", publish_topic_id, "imu_msg");
  nh.param<std::string> ("frame_id", imu.frame_id, "base_link");
  nh.param<bool>("use_enu",imu.enu_selected,false);

  nh.param<double> ("cov_x",cov_x,1.0);
  nh.param<double> ("cov_y",cov_y,1.0);
  nh.param<double> ("cov_z",cov_z,1.0);

  imu.set_covariance(cov_x,cov_y,cov_z);

  ros::Subscriber sub = nh.subscribe(subscribe_topic_id, 1,&VectorNav::processSerialCallback,&imu);
  imu.imu_pub = nh.advertise<sensor_msgs::Imu> (publish_topic_id, 1);

  ros::spin();

  return 0;

}
