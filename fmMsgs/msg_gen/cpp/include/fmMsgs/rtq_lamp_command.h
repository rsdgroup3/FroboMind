/* Auto-generated by genmsg_cpp for file /home/rsd/groovy_workspace/FroboMind-Fuerte/fmMsgs/msg/rtq_lamp_command.msg */
#ifndef FMMSGS_MESSAGE_RTQ_LAMP_COMMAND_H
#define FMMSGS_MESSAGE_RTQ_LAMP_COMMAND_H
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
struct rtq_lamp_command_ {
  typedef rtq_lamp_command_<ContainerAllocator> Type;

  rtq_lamp_command_()
  : header()
  , LampYellow(false)
  , LampRed(false)
  {
  }

  rtq_lamp_command_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , LampYellow(false)
  , LampRed(false)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef uint8_t _LampYellow_type;
  uint8_t LampYellow;

  typedef uint8_t _LampRed_type;
  uint8_t LampRed;


  typedef boost::shared_ptr< ::fmMsgs::rtq_lamp_command_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::fmMsgs::rtq_lamp_command_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct rtq_lamp_command
typedef  ::fmMsgs::rtq_lamp_command_<std::allocator<void> > rtq_lamp_command;

typedef boost::shared_ptr< ::fmMsgs::rtq_lamp_command> rtq_lamp_commandPtr;
typedef boost::shared_ptr< ::fmMsgs::rtq_lamp_command const> rtq_lamp_commandConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::fmMsgs::rtq_lamp_command_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::fmMsgs::rtq_lamp_command_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace fmMsgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::fmMsgs::rtq_lamp_command_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::fmMsgs::rtq_lamp_command_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::fmMsgs::rtq_lamp_command_<ContainerAllocator> > {
  static const char* value() 
  {
    return "04c36abb8ea69dddac691c50e1fb96f6";
  }

  static const char* value(const  ::fmMsgs::rtq_lamp_command_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x04c36abb8ea69dddULL;
  static const uint64_t static_value2 = 0xac691c50e1fb96f6ULL;
};

template<class ContainerAllocator>
struct DataType< ::fmMsgs::rtq_lamp_command_<ContainerAllocator> > {
  static const char* value() 
  {
    return "fmMsgs/rtq_lamp_command";
  }

  static const char* value(const  ::fmMsgs::rtq_lamp_command_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::fmMsgs::rtq_lamp_command_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
bool LampYellow\n\
bool LampRed\n\
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

  static const char* value(const  ::fmMsgs::rtq_lamp_command_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::fmMsgs::rtq_lamp_command_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::fmMsgs::rtq_lamp_command_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::fmMsgs::rtq_lamp_command_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.LampYellow);
    stream.next(m.LampRed);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct rtq_lamp_command_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::fmMsgs::rtq_lamp_command_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::fmMsgs::rtq_lamp_command_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "LampYellow: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.LampYellow);
    s << indent << "LampRed: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.LampRed);
  }
};


} // namespace message_operations
} // namespace ros

#endif // FMMSGS_MESSAGE_RTQ_LAMP_COMMAND_H

