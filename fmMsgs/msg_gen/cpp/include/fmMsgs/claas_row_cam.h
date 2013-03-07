/* Auto-generated by genmsg_cpp for file /home/rsd/groovy_workspace/FroboMind-Fuerte/fmMsgs/msg/claas_row_cam.msg */
#ifndef FMMSGS_MESSAGE_CLAAS_ROW_CAM_H
#define FMMSGS_MESSAGE_CLAAS_ROW_CAM_H
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
struct claas_row_cam_ {
  typedef claas_row_cam_<ContainerAllocator> Type;

  claas_row_cam_()
  : header()
  , quality(0)
  , heading(0)
  , offset(0)
  {
  }

  claas_row_cam_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , quality(0)
  , heading(0)
  , offset(0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef uint8_t _quality_type;
  uint8_t quality;

  typedef int16_t _heading_type;
  int16_t heading;

  typedef int16_t _offset_type;
  int16_t offset;


  typedef boost::shared_ptr< ::fmMsgs::claas_row_cam_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::fmMsgs::claas_row_cam_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct claas_row_cam
typedef  ::fmMsgs::claas_row_cam_<std::allocator<void> > claas_row_cam;

typedef boost::shared_ptr< ::fmMsgs::claas_row_cam> claas_row_camPtr;
typedef boost::shared_ptr< ::fmMsgs::claas_row_cam const> claas_row_camConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::fmMsgs::claas_row_cam_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::fmMsgs::claas_row_cam_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace fmMsgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::fmMsgs::claas_row_cam_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::fmMsgs::claas_row_cam_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::fmMsgs::claas_row_cam_<ContainerAllocator> > {
  static const char* value() 
  {
    return "55f1517c9516aa61e1abcd6b8e1baa07";
  }

  static const char* value(const  ::fmMsgs::claas_row_cam_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x55f1517c9516aa61ULL;
  static const uint64_t static_value2 = 0xe1abcd6b8e1baa07ULL;
};

template<class ContainerAllocator>
struct DataType< ::fmMsgs::claas_row_cam_<ContainerAllocator> > {
  static const char* value() 
  {
    return "fmMsgs/claas_row_cam";
  }

  static const char* value(const  ::fmMsgs::claas_row_cam_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::fmMsgs::claas_row_cam_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
uint8 quality\n\
int16 heading\n\
int16 offset\n\
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

  static const char* value(const  ::fmMsgs::claas_row_cam_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::fmMsgs::claas_row_cam_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::fmMsgs::claas_row_cam_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::fmMsgs::claas_row_cam_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.quality);
    stream.next(m.heading);
    stream.next(m.offset);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct claas_row_cam_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::fmMsgs::claas_row_cam_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::fmMsgs::claas_row_cam_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "quality: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.quality);
    s << indent << "heading: ";
    Printer<int16_t>::stream(s, indent + "  ", v.heading);
    s << indent << "offset: ";
    Printer<int16_t>::stream(s, indent + "  ", v.offset);
  }
};


} // namespace message_operations
} // namespace ros

#endif // FMMSGS_MESSAGE_CLAAS_ROW_CAM_H

