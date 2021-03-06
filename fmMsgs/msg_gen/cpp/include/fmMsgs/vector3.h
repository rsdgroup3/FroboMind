/* Auto-generated by genmsg_cpp for file /home/rsd/groovy_workspace/FroboMind-Fuerte/fmMsgs/msg/vector3.msg */
#ifndef FMMSGS_MESSAGE_VECTOR3_H
#define FMMSGS_MESSAGE_VECTOR3_H
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
struct vector3_ {
  typedef vector3_<ContainerAllocator> Type;

  vector3_()
  : header()
  , x(0.0)
  , y(0.0)
  , z(0.0)
  , phi(0.0)
  {
  }

  vector3_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , x(0.0)
  , y(0.0)
  , z(0.0)
  , phi(0.0)
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

  typedef double _phi_type;
  double phi;


  typedef boost::shared_ptr< ::fmMsgs::vector3_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::fmMsgs::vector3_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct vector3
typedef  ::fmMsgs::vector3_<std::allocator<void> > vector3;

typedef boost::shared_ptr< ::fmMsgs::vector3> vector3Ptr;
typedef boost::shared_ptr< ::fmMsgs::vector3 const> vector3ConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::fmMsgs::vector3_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::fmMsgs::vector3_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace fmMsgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::fmMsgs::vector3_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::fmMsgs::vector3_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::fmMsgs::vector3_<ContainerAllocator> > {
  static const char* value() 
  {
    return "0927916307bb98b0ef51a91a21d0fbf4";
  }

  static const char* value(const  ::fmMsgs::vector3_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x0927916307bb98b0ULL;
  static const uint64_t static_value2 = 0xef51a91a21d0fbf4ULL;
};

template<class ContainerAllocator>
struct DataType< ::fmMsgs::vector3_<ContainerAllocator> > {
  static const char* value() 
  {
    return "fmMsgs/vector3";
  }

  static const char* value(const  ::fmMsgs::vector3_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::fmMsgs::vector3_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 phi\n\
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

  static const char* value(const  ::fmMsgs::vector3_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::fmMsgs::vector3_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::fmMsgs::vector3_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::fmMsgs::vector3_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.x);
    stream.next(m.y);
    stream.next(m.z);
    stream.next(m.phi);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct vector3_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::fmMsgs::vector3_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::fmMsgs::vector3_<ContainerAllocator> & v) 
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
    s << indent << "phi: ";
    Printer<double>::stream(s, indent + "  ", v.phi);
  }
};


} // namespace message_operations
} // namespace ros

#endif // FMMSGS_MESSAGE_VECTOR3_H

