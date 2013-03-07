#include <stdio.h>
#include <string.h>
#include <sstream>
#include <vector>
#include "ros/ros.h"
#include "fmMsgs/nmea.h"
#include "fmMsgs/serial.h"
#include "boost/tokenizer.hpp"
#include "boost/lexical_cast.hpp"
#include "boost/algorithm/string.hpp"

#define ASCII_OFFSET '0'

ros::Publisher str_to_msg_pub;
ros::Publisher msg_to_str_pub;
fmMsgs::nmea nmea_msg;
bool use_checksum;


typedef boost::tokenizer<boost::char_separator<char> > tokenizer;

/* Parses from a string into two hex characters */
int atox(const char *s) {
	int ret = 0;
	int cnt = 8;
	while (cnt) {
		if ((*s >= '0') && (*s <= '9')) {
			ret <<= 4;
			ret += (*s - '0');
			cnt--;
			s++;
			continue;
		}
		if ((*s >= 'a') && (*s <= 'f')) {
			ret <<= 4;
			ret += (*s - 'a' + 10);
			cnt--;
			s++;
			continue;
		}
		if ((*s >= 'A') && (*s <= 'F')) {
			ret <<= 4;
			ret += (*s - 'A' + 10);
			cnt--;
			s++;
			continue;
		}
		break;
	}
	return ret;
}

/* Calculates expected checksum and compares to contained checksum */
bool verify_checksum(std::vector<std::string> tokens) {
	//Setup proper string
	std::string temp_str = "0x";
	temp_str.append(tokens.at(tokens.size() - 1).c_str());

	unsigned char expected_checksum = atox(
			tokens.at(tokens.size() - 1).c_str());
	unsigned char checksum = 0;
	unsigned char *string_breakdown;

	//Calculate check sum
	for (unsigned int i = 0; i < tokens.size() - 1; i++) {
		string_breakdown = (unsigned char*) tokens.at(i).c_str();

		for (unsigned int j = 0; j < tokens.at(i).length(); j++)
			if (string_breakdown[j] != '$')
				checksum ^= string_breakdown[j];

		//Remember the ','
		if (i < (tokens.size() - 2))
			checksum ^= ',';
	}

	return (checksum == expected_checksum);
}

/* Parses from serial message to nmea message */
void str_to_msg_callback(const fmMsgs::serial::ConstPtr& msg) {
	std::string nmeastr(msg->data);
	boost::char_separator<char> sep("$,*\r");
	tokenizer tokens(nmeastr, sep);
	std::vector<std::string> nmea;

	nmea.assign(tokens.begin(), tokens.end());
	try {

		//build message
		nmea_msg.header.stamp = ros::Time::now();
		nmea_msg.id =  nmea.at(0).substr(0,2);
		nmea_msg.type = nmea.at(0).substr(2,3);
		nmea_msg.data.clear();
		for (int i = 1 ; i < nmea.size() ; i++ )
			nmea_msg.data.push_back( nmea.at(i).c_str() );

		if(use_checksum)
		{
			if (!verify_checksum(nmea))
			{
				ROS_WARN("NMEA string discarded due to faulty checksum");
			}
			else
				//publish message
				str_to_msg_pub.publish(nmea_msg);
		}
		else
			//publish message
			str_to_msg_pub.publish(nmea_msg);

	} catch (boost::bad_lexical_cast &) {
		ROS_WARN("nmea_parser: bad lexical cast");
	}
}

/* Parses from  nmea message to serial message */
void msg_to_str_callback(const fmMsgs::nmea::ConstPtr& msg) {
	std::vector<std::string> data_list;
	std::vector<std::string> nmea_list;
	std::string data_string;
	fmMsgs::serial nmea_string;

	//construct data string
	for(int i = 0 ; i < msg->data.size() ; i++ )
		data_list.push_back(msg->data[i]);
	data_string = boost::algorithm::join(data_list, ",");

	//construct nmea string
	nmea_list.push_back("$");
	nmea_list.push_back(msg->id);
	nmea_list.push_back(msg->type);
	nmea_list.push_back(",");
	nmea_list.push_back(data_string);
	nmea_list.push_back("*");
	nmea_list.push_back("00");
	nmea_list.push_back("\r");

	nmea_string.header.stamp = msg->header.stamp;
	nmea_string.data = boost::algorithm::join(nmea_list,"");

	msg_to_str_pub.publish(nmea_string);
}


/* sets up nmea parser node */
int main(int argc, char **argv) {
	ros::init(argc, argv, "nmea_parser");
	ros::NodeHandle nh;
	ros::NodeHandle n("~");

	std::string str_to_msg_sub_id;
	std::string str_to_msg_pub_id;
	std::string msg_to_str_sub_id;
	std::string msg_to_str_pub_id;

	n.param<std::string>("str_to_msg_sub", str_to_msg_sub_id,
			"/fmCSP/S0_rx_msg");
	n.param<std::string>("str_to_msg_pub", str_to_msg_pub_id,
			"/nmea_in");
	n.param<std::string>("msg_to_str_sub", msg_to_str_sub_id,
			"/nmea_out");
	n.param<std::string>("msg_to_str_pub", msg_to_str_pub_id,
			"/fmCSP/S0_tx_msg");
	n.param<bool>("use_nmea_checksum" , use_checksum , false);

	ros::Subscriber str_to_msg_sub = nh.subscribe(str_to_msg_sub_id, 10, str_to_msg_callback);
	str_to_msg_pub = nh.advertise<fmMsgs::nmea>(str_to_msg_pub_id, 1);

	ros::Subscriber msg_to_str_sub = nh.subscribe(msg_to_str_sub_id, 10, msg_to_str_callback);
	msg_to_str_pub = nh.advertise<fmMsgs::serial>(msg_to_str_pub_id, 1);

	ros::spin();
	return 0;
}

