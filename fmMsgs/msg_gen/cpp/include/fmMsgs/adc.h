/* Auto-generated by genmsg_cpp for file /home/rsd/groovy_workspace/FroboMind-Fuerte/fmMsgs/msg/adc.msg */
#ifndef FMMSGS_MESSAGE_ADC_H
#define FMMSGS_MESSAGE_ADC_H
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
struct adc_ {
  typedef adc_<ContainerAllocator> Type;

  adc_()
  : header()
  , value0(0.0)
  , value1(0.0)
  {
  }

  adc_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , value0(0.0)
  , value1(0.0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef double _value0_type;
  double value0;

  typedef double _value1_type;
  double value1;


  typedef boost::shared_ptr< ::fmMsgs::adc_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::fmMsgs::adc_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct adc
typedef  ::fmMsgs::adc_<std::allocator<void> > adc;

typedef boost::shared_ptr< ::fmMsgs::adc> adcPtr;
typedef boost::shared_ptr< ::fmMsgs::adc const> adcConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::fmMsgs::adc_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::fmMsgs::adc_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace fmMsgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::fmMsgs::adc_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::fmMsgs::adc_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::fmMsgs::adc_<ContainerAllocator> > {
  static const char* value() 
  {
    return "8936c758e4769aa790b21929da850218";
  }

  static const char* value(const  ::fmMsgs::adc_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x8936c758e4769aa7ULL;
  static const uint64_t static_value2 = 0x90b21929da850218ULL;
};

template<class ContainerAllocator>
struct DataType< ::fmMsgs::adc_<ContainerAllocator> > {
  static const char* value() 
  {
    return "fmMsgs/adc";
  }

  static const char* value(const  ::fmMsgs::adc_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::fmMsgs::adc_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
float64 value0\n\
float64 value1\n\
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

  static const char* value(const  ::fmMsgs::adc_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::fmMsgs::adc_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::fmMsgs::adc_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::fmMsgs::adc_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.value0);
    stream.next(m.value1);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct adc_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::fmMsgs::adc_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::fmMsgs::adc_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "value0: ";
    Printer<double>::stream(s, indent + "  ", v.value0);
    s << indent << "value1: ";
    Printer<double>::stream(s, indent + "  ", v.value1);
  }
};


} // namespace message_operations
} // namespace ros

#endif // FMMSGS_MESSAGE_ADC_H
