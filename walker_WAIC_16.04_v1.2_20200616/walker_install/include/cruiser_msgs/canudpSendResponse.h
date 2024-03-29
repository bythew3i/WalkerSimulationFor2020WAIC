// Generated by gencpp from file cruiser_msgs/canudpSendResponse.msg
// DO NOT EDIT!


#ifndef CRUISER_MSGS_MESSAGE_CANUDPSENDRESPONSE_H
#define CRUISER_MSGS_MESSAGE_CANUDPSENDRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace cruiser_msgs
{
template <class ContainerAllocator>
struct canudpSendResponse_
{
  typedef canudpSendResponse_<ContainerAllocator> Type;

  canudpSendResponse_()
    : time(0)  {
    }
  canudpSendResponse_(const ContainerAllocator& _alloc)
    : time(0)  {
  (void)_alloc;
    }



   typedef uint32_t _time_type;
  _time_type time;





  typedef boost::shared_ptr< ::cruiser_msgs::canudpSendResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cruiser_msgs::canudpSendResponse_<ContainerAllocator> const> ConstPtr;

}; // struct canudpSendResponse_

typedef ::cruiser_msgs::canudpSendResponse_<std::allocator<void> > canudpSendResponse;

typedef boost::shared_ptr< ::cruiser_msgs::canudpSendResponse > canudpSendResponsePtr;
typedef boost::shared_ptr< ::cruiser_msgs::canudpSendResponse const> canudpSendResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cruiser_msgs::canudpSendResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cruiser_msgs::canudpSendResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace cruiser_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'nav_msgs': ['/opt/ros/kinetic/share/nav_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'cruiser_msgs': ['/home/cjl/core_ws/walker_ws/walker2_motion_output/src/ros_common/cruiser_msgs/msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::cruiser_msgs::canudpSendResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cruiser_msgs::canudpSendResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cruiser_msgs::canudpSendResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cruiser_msgs::canudpSendResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cruiser_msgs::canudpSendResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cruiser_msgs::canudpSendResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cruiser_msgs::canudpSendResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d16de83ef4e326bbfdc1e90377f0a2c6";
  }

  static const char* value(const ::cruiser_msgs::canudpSendResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd16de83ef4e326bbULL;
  static const uint64_t static_value2 = 0xfdc1e90377f0a2c6ULL;
};

template<class ContainerAllocator>
struct DataType< ::cruiser_msgs::canudpSendResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cruiser_msgs/canudpSendResponse";
  }

  static const char* value(const ::cruiser_msgs::canudpSendResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cruiser_msgs::canudpSendResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
uint32 time\n\
\n\
\n\
";
  }

  static const char* value(const ::cruiser_msgs::canudpSendResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cruiser_msgs::canudpSendResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.time);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct canudpSendResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cruiser_msgs::canudpSendResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cruiser_msgs::canudpSendResponse_<ContainerAllocator>& v)
  {
    s << indent << "time: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.time);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CRUISER_MSGS_MESSAGE_CANUDPSENDRESPONSE_H
