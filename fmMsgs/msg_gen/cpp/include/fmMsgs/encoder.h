/* Auto-generated by genmsg_cpp for file /home/rsd/groovy_workspace/FroboMind-Fuerte/fmMsgs/msg/encoder.msg */
#ifndef FMMSGS_MESSAGE_ENCODER_H
#define FMMSGS_MESSAGE_ENCODER_H
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
struct encoder_ {
  typedef encoder_<ContainerAllocator> Type;

  encoder_()
  : header()
  , encoderticks(0)
  {
  }

  encoder_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , encoderticks(0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef int32_t _encoderticks_type;
  int32_t encoderticks;


  typedef boost::shared_ptr< ::fmMsgs::encoder_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::fmMsgs::encoder_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct encoder
typedef  ::fmMsgs::encoder_<std::allocator<void> > encoder;

typedef boost::shared_ptr< ::fmMsgs::encoder> encoderPtr;
typedef boost::shared_ptr< ::fmMsgs::encoder const> encoderConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::fmMsgs::encoder_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::fmMsgs::encoder_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace fmMsgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::fmMsgs::encoder_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::fmMsgs::encoder_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::fmMsgs::encoder_<ContainerAllocator> > {
  static const char* value() 
  {
    return "26ceb5911039b0ed02c96fff91a554fc";
  }

  static const char* value(const  ::fmMsgs::encoder_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x26ceb5911039b0edULL;
  static const uint64_t static_value2 = 0x02c96fff91a554fcULL;
};

template<class ContainerAllocator>
struct DataType< ::fmMsgs::encoder_<ContainerAllocator> > {
  static const char* value() 
  {
    return "fmMsgs/encoder";
  }

  static const char* value(const  ::fmMsgs::encoder_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::fmMsgs::encoder_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
int32 encoderticks\n\
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

  static const char* value(const  ::fmMsgs::encoder_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::fmMsgs::encoder_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::fmMsgs::encoder_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::fmMsgs::encoder_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.encoderticks);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct encoder_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::fmMsgs::encoder_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::fmMsgs::encoder_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "encoderticks: ";
    Printer<int32_t>::stream(s, indent + "  ", v.encoderticks);
  }
};


} // namespace message_operations
} // namespace ros

#endif // FMMSGS_MESSAGE_ENCODER_H

