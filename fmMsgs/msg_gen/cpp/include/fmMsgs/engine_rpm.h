/* Auto-generated by genmsg_cpp for file /home/rsd/groovy_workspace/FroboMind-Fuerte/fmMsgs/msg/engine_rpm.msg */
#ifndef FMMSGS_MESSAGE_ENGINE_RPM_H
#define FMMSGS_MESSAGE_ENGINE_RPM_H
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
struct engine_rpm_ {
  typedef engine_rpm_<ContainerAllocator> Type;

  engine_rpm_()
  : header()
  , rpm(0.0)
  {
  }

  engine_rpm_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , rpm(0.0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef double _rpm_type;
  double rpm;


  typedef boost::shared_ptr< ::fmMsgs::engine_rpm_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::fmMsgs::engine_rpm_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct engine_rpm
typedef  ::fmMsgs::engine_rpm_<std::allocator<void> > engine_rpm;

typedef boost::shared_ptr< ::fmMsgs::engine_rpm> engine_rpmPtr;
typedef boost::shared_ptr< ::fmMsgs::engine_rpm const> engine_rpmConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::fmMsgs::engine_rpm_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::fmMsgs::engine_rpm_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace fmMsgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::fmMsgs::engine_rpm_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::fmMsgs::engine_rpm_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::fmMsgs::engine_rpm_<ContainerAllocator> > {
  static const char* value() 
  {
    return "227ff304119c584c2200d7d923efaba9";
  }

  static const char* value(const  ::fmMsgs::engine_rpm_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x227ff304119c584cULL;
  static const uint64_t static_value2 = 0x2200d7d923efaba9ULL;
};

template<class ContainerAllocator>
struct DataType< ::fmMsgs::engine_rpm_<ContainerAllocator> > {
  static const char* value() 
  {
    return "fmMsgs/engine_rpm";
  }

  static const char* value(const  ::fmMsgs::engine_rpm_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::fmMsgs::engine_rpm_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
float64 rpm\n\
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

  static const char* value(const  ::fmMsgs::engine_rpm_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::fmMsgs::engine_rpm_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::fmMsgs::engine_rpm_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::fmMsgs::engine_rpm_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.rpm);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct engine_rpm_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::fmMsgs::engine_rpm_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::fmMsgs::engine_rpm_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "rpm: ";
    Printer<double>::stream(s, indent + "  ", v.rpm);
  }
};


} // namespace message_operations
} // namespace ros

#endif // FMMSGS_MESSAGE_ENGINE_RPM_H
