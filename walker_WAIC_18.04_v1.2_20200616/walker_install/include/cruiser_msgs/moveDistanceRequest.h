// Generated by gencpp from file cruiser_msgs/moveDistanceRequest.msg
// DO NOT EDIT!


#ifndef CRUISER_MSGS_MESSAGE_MOVEDISTANCEREQUEST_H
#define CRUISER_MSGS_MESSAGE_MOVEDISTANCEREQUEST_H


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
struct moveDistanceRequest_
{
  typedef moveDistanceRequest_<ContainerAllocator> Type;

  moveDistanceRequest_()
    : reserved(0)  {
    }
  moveDistanceRequest_(const ContainerAllocator& _alloc)
    : reserved(0)  {
  (void)_alloc;
    }



   typedef uint32_t _reserved_type;
  _reserved_type reserved;





  typedef boost::shared_ptr< ::cruiser_msgs::moveDistanceRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cruiser_msgs::moveDistanceRequest_<ContainerAllocator> const> ConstPtr;

}; // struct moveDistanceRequest_

typedef ::cruiser_msgs::moveDistanceRequest_<std::allocator<void> > moveDistanceRequest;

typedef boost::shared_ptr< ::cruiser_msgs::moveDistanceRequest > moveDistanceRequestPtr;
typedef boost::shared_ptr< ::cruiser_msgs::moveDistanceRequest const> moveDistanceRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cruiser_msgs::moveDistanceRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cruiser_msgs::moveDistanceRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::cruiser_msgs::moveDistanceRequest_<ContainerAllocator1> & lhs, const ::cruiser_msgs::moveDistanceRequest_<ContainerAllocator2> & rhs)
{
  return lhs.reserved == rhs.reserved;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::cruiser_msgs::moveDistanceRequest_<ContainerAllocator1> & lhs, const ::cruiser_msgs::moveDistanceRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace cruiser_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::cruiser_msgs::moveDistanceRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cruiser_msgs::moveDistanceRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cruiser_msgs::moveDistanceRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cruiser_msgs::moveDistanceRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cruiser_msgs::moveDistanceRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cruiser_msgs::moveDistanceRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cruiser_msgs::moveDistanceRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b074c9ce6b42f25fa45f883d993a830a";
  }

  static const char* value(const ::cruiser_msgs::moveDistanceRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb074c9ce6b42f25fULL;
  static const uint64_t static_value2 = 0xa45f883d993a830aULL;
};

template<class ContainerAllocator>
struct DataType< ::cruiser_msgs::moveDistanceRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cruiser_msgs/moveDistanceRequest";
  }

  static const char* value(const ::cruiser_msgs::moveDistanceRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cruiser_msgs::moveDistanceRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint32 reserved\n"
;
  }

  static const char* value(const ::cruiser_msgs::moveDistanceRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cruiser_msgs::moveDistanceRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.reserved);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct moveDistanceRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cruiser_msgs::moveDistanceRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cruiser_msgs::moveDistanceRequest_<ContainerAllocator>& v)
  {
    s << indent << "reserved: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.reserved);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CRUISER_MSGS_MESSAGE_MOVEDISTANCEREQUEST_H