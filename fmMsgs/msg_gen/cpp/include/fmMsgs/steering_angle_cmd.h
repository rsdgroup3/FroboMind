/* Auto-generated by genmsg_cpp for file /home/rsd/groovy_workspace/FroboMind-Fuerte/fmMsgs/msg/steering_angle_cmd.msg */
#ifndef FMMSGS_MESSAGE_STEERING_ANGLE_CMD_H
#define FMMSGS_MESSAGE_STEERING_ANGLE_CMD_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "std_msgs/Header.h"

namespace fmMsgs
{
template <class ContainerAllocator>
struct steering_angle_cmd_ {
  typedef steering_angle_cmd_<ContainerAllocator> Type;

  steering_angle_cmd_()
  : header()
  , steering_angle(0.0)
  {
  }

  steering_angle_cmd_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , steering_angle(0.0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef double _steering_angle_type;
  double steering_angle;


  typedef boost::shared_ptr< ::fmMsgs::steering_angle_cmd_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::fmMsgs::steering_angle_cmd_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct steering_angle_cmd
typedef  ::fmMsgs::steering_angle_cmd_<std::allocator<void> > steering_angle_cmd;

typedef boost::shared_ptr< ::fmMsgs::steering_angle_cmd> steering_angle_cmdPtr;
typedef boost::shared_ptr< ::fmMsgs::steering_angle_cmd const> steering_angle_cmdConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::fmMsgs::steering_angle_cmd_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::fmMsgs::steering_angle_cmd_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace fmMsgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::fmMsgs::steering_angle_cmd_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::fmMsgs::steering_angle_cmd_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::fmMsgs::steering_angle_cmd_<ContainerAllocator> > {
  static const char* value() 
  {
    return "90da1b169b320e4c6ce83d800d2523d4";
  }

  static const char* value(const  ::fmMsgs::steering_angle_cmd_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x90da1b169b320e4cULL;
  static const uint64_t static_value2 = 0x6ce83d800d2523d4ULL;
};

template<class ContainerAllocator>
struct DataType< ::fmMsgs::steering_angle_cmd_<ContainerAllocator> > {
  static const char* value() 
  {
    return "fmMsgs/steering_angle_cmd";
  }

  static const char* value(const  ::fmMsgs::steering_angle_cmd_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::fmMsgs::steering_angle_cmd_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
float64 steering_angle\n\
\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
";
  }

  static const char* value(const  ::fmMsgs::steering_angle_cmd_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::fmMsgs::steering_angle_cmd_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::fmMsgs::steering_angle_cmd_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::fmMsgs::steering_angle_cmd_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.steering_angle);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct steering_angle_cmd_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::fmMsgs::steering_angle_cmd_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::fmMsgs::steering_angle_cmd_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "steering_angle: ";
    Printer<double>::stream(s, indent + "  ", v.steering_angle);
  }
};


} // namespace message_operations
} // namespace ros

#endif // FMMSGS_MESSAGE_STEERING_ANGLE_CMD_H

