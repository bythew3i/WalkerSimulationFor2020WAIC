// Generated by gencpp from file cruiser_msgs/CruiserJointSetting.msg
// DO NOT EDIT!


#ifndef CRUISER_MSGS_MESSAGE_CRUISERJOINTSETTING_H
#define CRUISER_MSGS_MESSAGE_CRUISERJOINTSETTING_H


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
struct CruiserJointSetting_
{
  typedef CruiserJointSetting_<ContainerAllocator> Type;

  CruiserJointSetting_()
    : joint_num(0)
    , cmd()
    , jointIndex()
    , parameter()  {
    }
  CruiserJointSetting_(const ContainerAllocator& _alloc)
    : joint_num(0)
    , cmd(_alloc)
    , jointIndex(_alloc)
    , parameter(_alloc)  {
  (void)_alloc;
    }



   typedef int16_t _joint_num_type;
  _joint_num_type joint_num;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _cmd_type;
  _cmd_type cmd;

   typedef std::vector<uint32_t, typename ContainerAllocator::template rebind<uint32_t>::other >  _jointIndex_type;
  _jointIndex_type jointIndex;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _parameter_type;
  _parameter_type parameter;





  typedef boost::shared_ptr< ::cruiser_msgs::CruiserJointSetting_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cruiser_msgs::CruiserJointSetting_<ContainerAllocator> const> ConstPtr;

}; // struct CruiserJointSetting_

typedef ::cruiser_msgs::CruiserJointSetting_<std::allocator<void> > CruiserJointSetting;

typedef boost::shared_ptr< ::cruiser_msgs::CruiserJointSetting > CruiserJointSettingPtr;
typedef boost::shared_ptr< ::cruiser_msgs::CruiserJointSetting const> CruiserJointSettingConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cruiser_msgs::CruiserJointSetting_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cruiser_msgs::CruiserJointSetting_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace cruiser_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'nav_msgs': ['/opt/ros/kinetic/share/nav_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'cruiser_msgs': ['/home/cjl/core_ws/walker_ws/walker2_motion_output/src/ros_common/cruiser_msgs/msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::cruiser_msgs::CruiserJointSetting_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cruiser_msgs::CruiserJointSetting_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cruiser_msgs::CruiserJointSetting_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cruiser_msgs::CruiserJointSetting_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cruiser_msgs::CruiserJointSetting_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cruiser_msgs::CruiserJointSetting_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cruiser_msgs::CruiserJointSetting_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2cb5800afbbffbf910bd3074e8d992fb";
  }

  static const char* value(const ::cruiser_msgs::CruiserJointSetting_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2cb5800afbbffbf9ULL;
  static const uint64_t static_value2 = 0x10bd3074e8d992fbULL;
};

template<class ContainerAllocator>
struct DataType< ::cruiser_msgs::CruiserJointSetting_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cruiser_msgs/CruiserJointSetting";
  }

  static const char* value(const ::cruiser_msgs::CruiserJointSetting_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cruiser_msgs::CruiserJointSetting_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int16 joint_num\n\
\n\
# Joints name to control in array\n\
# example - [\"stop\", \"set Kp\", \"read zero\"]\n\
string cmd\n\
\n\
# Joints index to control in array\n\
uint32[] jointIndex\n\
\n\
# Corresponding joints postion\n\
# unit - radian;\n\
# example - [0.54, 1.22, 1.39]\n\
float64[] parameter\n\
\n\
\n\
\n\
";
  }

  static const char* value(const ::cruiser_msgs::CruiserJointSetting_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cruiser_msgs::CruiserJointSetting_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.joint_num);
      stream.next(m.cmd);
      stream.next(m.jointIndex);
      stream.next(m.parameter);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CruiserJointSetting_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cruiser_msgs::CruiserJointSetting_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cruiser_msgs::CruiserJointSetting_<ContainerAllocator>& v)
  {
    s << indent << "joint_num: ";
    Printer<int16_t>::stream(s, indent + "  ", v.joint_num);
    s << indent << "cmd: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.cmd);
    s << indent << "jointIndex[]" << std::endl;
    for (size_t i = 0; i < v.jointIndex.size(); ++i)
    {
      s << indent << "  jointIndex[" << i << "]: ";
      Printer<uint32_t>::stream(s, indent + "  ", v.jointIndex[i]);
    }
    s << indent << "parameter[]" << std::endl;
    for (size_t i = 0; i < v.parameter.size(); ++i)
    {
      s << indent << "  parameter[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.parameter[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // CRUISER_MSGS_MESSAGE_CRUISERJOINTSETTING_H
