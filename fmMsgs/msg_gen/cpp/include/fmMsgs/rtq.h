/* Auto-generated by genmsg_cpp for file /home/rsd/groovy_workspace/FroboMind-Fuerte/fmMsgs/msg/rtq.msg */
#ifndef FMMSGS_MESSAGE_RTQ_H
#define FMMSGS_MESSAGE_RTQ_H
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
struct rtq_ {
  typedef rtq_<ContainerAllocator> Type;

  rtq_()
  : header()
  , BrushlessCounter(0)
  , BrushlessCounterRelative(0)
  , BatteryVoltage(0.0)
  , BatteryAmpere(0.0)
  , TrackDistance(0.0)
  , TrackDistanceRelative(0.0)
  {
  }

  rtq_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , BrushlessCounter(0)
  , BrushlessCounterRelative(0)
  , BatteryVoltage(0.0)
  , BatteryAmpere(0.0)
  , TrackDistance(0.0)
  , TrackDistanceRelative(0.0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef int32_t _BrushlessCounter_type;
  int32_t BrushlessCounter;

  typedef int32_t _BrushlessCounterRelative_type;
  int32_t BrushlessCounterRelative;

  typedef double _BatteryVoltage_type;
  double BatteryVoltage;

  typedef double _BatteryAmpere_type;
  double BatteryAmpere;

  typedef double _TrackDistance_type;
  double TrackDistance;

  typedef double _TrackDistanceRelative_type;
  double TrackDistanceRelative;


  typedef boost::shared_ptr< ::fmMsgs::rtq_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::fmMsgs::rtq_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct rtq
typedef  ::fmMsgs::rtq_<std::allocator<void> > rtq;

typedef boost::shared_ptr< ::fmMsgs::rtq> rtqPtr;
typedef boost::shared_ptr< ::fmMsgs::rtq const> rtqConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::fmMsgs::rtq_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::fmMsgs::rtq_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace fmMsgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::fmMsgs::rtq_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::fmMsgs::rtq_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::fmMsgs::rtq_<ContainerAllocator> > {
  static const char* value() 
  {
    return "5f3a49abafa7ac778f37c5bada73e588";
  }

  static const char* value(const  ::fmMsgs::rtq_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x5f3a49abafa7ac77ULL;
  static const uint64_t static_value2 = 0x8f37c5bada73e588ULL;
};

template<class ContainerAllocator>
struct DataType< ::fmMsgs::rtq_<ContainerAllocator> > {
  static const char* value() 
  {
    return "fmMsgs/rtq";
  }

  static const char* value(const  ::fmMsgs::rtq_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::fmMsgs::rtq_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
int32 BrushlessCounter\n\
int32 BrushlessCounterRelative\n\
float64 BatteryVoltage\n\
float64 BatteryAmpere\n\
float64 TrackDistance\n\
float64 TrackDistanceRelative\n\
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

  static const char* value(const  ::fmMsgs::rtq_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::fmMsgs::rtq_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::fmMsgs::rtq_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::fmMsgs::rtq_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.BrushlessCounter);
    stream.next(m.BrushlessCounterRelative);
    stream.next(m.BatteryVoltage);
    stream.next(m.BatteryAmpere);
    stream.next(m.TrackDistance);
    stream.next(m.TrackDistanceRelative);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct rtq_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::fmMsgs::rtq_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::fmMsgs::rtq_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "BrushlessCounter: ";
    Printer<int32_t>::stream(s, indent + "  ", v.BrushlessCounter);
    s << indent << "BrushlessCounterRelative: ";
    Printer<int32_t>::stream(s, indent + "  ", v.BrushlessCounterRelative);
    s << indent << "BatteryVoltage: ";
    Printer<double>::stream(s, indent + "  ", v.BatteryVoltage);
    s << indent << "BatteryAmpere: ";
    Printer<double>::stream(s, indent + "  ", v.BatteryAmpere);
    s << indent << "TrackDistance: ";
    Printer<double>::stream(s, indent + "  ", v.TrackDistance);
    s << indent << "TrackDistanceRelative: ";
    Printer<double>::stream(s, indent + "  ", v.TrackDistanceRelative);
  }
};


} // namespace message_operations
} // namespace ros

#endif // FMMSGS_MESSAGE_RTQ_H

