/* Auto-generated by genmsg_cpp for file /home/rsd/groovy_workspace/FroboMind-Fuerte/fmMsgs/msg/magnetometer.msg */
#ifndef FMMSGS_MESSAGE_MAGNETOMETER_H
#define FMMSGS_MESSAGE_MAGNETOMETER_H
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
struct magnetometer_ {
  typedef magnetometer_<ContainerAllocator> Type;

  magnetometer_()
  : header()
  , x(0.0)
  , y(0.0)
  , z(0.0)
  {
  }

  magnetometer_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , x(0.0)
  , y(0.0)
  , z(0.0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef double _x_type;
  double x;

  typedef double _y_type;
  double y;

  typedef double _z_type;
  double z;


  typedef boost::shared_ptr< ::fmMsgs::magnetometer_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::fmMsgs::magnetometer_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct magnetometer
typedef  ::fmMsgs::magnetometer_<std::allocator<void> > magnetometer;

typedef boost::shared_ptr< ::fmMsgs::magnetometer> magnetometerPtr;
typedef boost::shared_ptr< ::fmMsgs::magnetometer const> magnetometerConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::fmMsgs::magnetometer_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::fmMsgs::magnetometer_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace fmMsgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::fmMsgs::magnetometer_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::fmMsgs::magnetometer_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::fmMsgs::magnetometer_<ContainerAllocator> > {
  static const char* value() 
  {
    return "5defbd163657b4f6f639ba6d5676cc01";
  }

  static const char* value(const  ::fmMsgs::magnetometer_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x5defbd163657b4f6ULL;
  static const uint64_t static_value2 = 0xf639ba6d5676cc01ULL;
};

template<class ContainerAllocator>
struct DataType< ::fmMsgs::magnetometer_<ContainerAllocator> > {
  static const char* value() 
  {
    return "fmMsgs/magnetometer";
  }

  static const char* value(const  ::fmMsgs::magnetometer_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::fmMsgs::magnetometer_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
float64 x\n\
float64 y\n\
float64 z\n\
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

  static const char* value(const  ::fmMsgs::magnetometer_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::fmMsgs::magnetometer_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::fmMsgs::magnetometer_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::fmMsgs::magnetometer_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.x);
    stream.next(m.y);
    stream.next(m.z);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct magnetometer_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::fmMsgs::magnetometer_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::fmMsgs::magnetometer_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<double>::stream(s, indent + "  ", v.z);
  }
};


} // namespace message_operations
} // namespace ros

#endif // FMMSGS_MESSAGE_MAGNETOMETER_H
