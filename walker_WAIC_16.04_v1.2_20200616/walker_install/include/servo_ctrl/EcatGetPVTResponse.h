// Generated by gencpp from file servo_ctrl/EcatGetPVTResponse.msg
// DO NOT EDIT!


#ifndef SERVO_CTRL_MESSAGE_ECATGETPVTRESPONSE_H
#define SERVO_CTRL_MESSAGE_ECATGETPVTRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace servo_ctrl
{
template <class ContainerAllocator>
struct EcatGetPVTResponse_
{
  typedef EcatGetPVTResponse_<ContainerAllocator> Type;

  EcatGetPVTResponse_()
    : pos(0.0)
    , vel(0.0)
    , trq(0.0)
    , errcode(0)  {
    }
  EcatGetPVTResponse_(const ContainerAllocator& _alloc)
    : pos(0.0)
    , vel(0.0)
    , trq(0.0)
    , errcode(0)  {
  (void)_alloc;
    }



   typedef float _pos_type;
  _pos_type pos;

   typedef float _vel_type;
  _vel_type vel;

   typedef float _trq_type;
  _trq_type trq;

   typedef uint16_t _errcode_type;
  _errcode_type errcode;





  typedef boost::shared_ptr< ::servo_ctrl::EcatGetPVTResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::servo_ctrl::EcatGetPVTResponse_<ContainerAllocator> const> ConstPtr;

}; // struct EcatGetPVTResponse_

typedef ::servo_ctrl::EcatGetPVTResponse_<std::allocator<void> > EcatGetPVTResponse;

typedef boost::shared_ptr< ::servo_ctrl::EcatGetPVTResponse > EcatGetPVTResponsePtr;
typedef boost::shared_ptr< ::servo_ctrl::EcatGetPVTResponse const> EcatGetPVTResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::servo_ctrl::EcatGetPVTResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::servo_ctrl::EcatGetPVTResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace servo_ctrl

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::servo_ctrl::EcatGetPVTResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::servo_ctrl::EcatGetPVTResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::servo_ctrl::EcatGetPVTResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::servo_ctrl::EcatGetPVTResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::servo_ctrl::EcatGetPVTResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::servo_ctrl::EcatGetPVTResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::servo_ctrl::EcatGetPVTResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0607ae32f3876e7e57f5a31f95f51d33";
  }

  static const char* value(const ::servo_ctrl::EcatGetPVTResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0607ae32f3876e7eULL;
  static const uint64_t static_value2 = 0x57f5a31f95f51d33ULL;
};

template<class ContainerAllocator>
struct DataType< ::servo_ctrl::EcatGetPVTResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "servo_ctrl/EcatGetPVTResponse";
  }

  static const char* value(const ::servo_ctrl::EcatGetPVTResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::servo_ctrl::EcatGetPVTResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 pos\n\
float32 vel\n\
float32 trq\n\
uint16 errcode\n\
\n\
";
  }

  static const char* value(const ::servo_ctrl::EcatGetPVTResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::servo_ctrl::EcatGetPVTResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.pos);
      stream.next(m.vel);
      stream.next(m.trq);
      stream.next(m.errcode);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct EcatGetPVTResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::servo_ctrl::EcatGetPVTResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::servo_ctrl::EcatGetPVTResponse_<ContainerAllocator>& v)
  {
    s << indent << "pos: ";
    Printer<float>::stream(s, indent + "  ", v.pos);
    s << indent << "vel: ";
    Printer<float>::stream(s, indent + "  ", v.vel);
    s << indent << "trq: ";
    Printer<float>::stream(s, indent + "  ", v.trq);
    s << indent << "errcode: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.errcode);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SERVO_CTRL_MESSAGE_ECATGETPVTRESPONSE_H
