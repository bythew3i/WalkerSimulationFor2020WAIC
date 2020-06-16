// Generated by gencpp from file cruiser_msgs/motion_srv.msg
// DO NOT EDIT!


#ifndef CRUISER_MSGS_MESSAGE_MOTION_SRV_H
#define CRUISER_MSGS_MESSAGE_MOTION_SRV_H

#include <ros/service_traits.h>


#include <cruiser_msgs/motion_srvRequest.h>
#include <cruiser_msgs/motion_srvResponse.h>


namespace cruiser_msgs
{

struct motion_srv
{

typedef motion_srvRequest Request;
typedef motion_srvResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct motion_srv
} // namespace cruiser_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::cruiser_msgs::motion_srv > {
  static const char* value()
  {
    return "e17f72359ddf3560f0cea3ec62c3ae72";
  }

  static const char* value(const ::cruiser_msgs::motion_srv&) { return value(); }
};

template<>
struct DataType< ::cruiser_msgs::motion_srv > {
  static const char* value()
  {
    return "cruiser_msgs/motion_srv";
  }

  static const char* value(const ::cruiser_msgs::motion_srv&) { return value(); }
};


// service_traits::MD5Sum< ::cruiser_msgs::motion_srvRequest> should match 
// service_traits::MD5Sum< ::cruiser_msgs::motion_srv > 
template<>
struct MD5Sum< ::cruiser_msgs::motion_srvRequest>
{
  static const char* value()
  {
    return MD5Sum< ::cruiser_msgs::motion_srv >::value();
  }
  static const char* value(const ::cruiser_msgs::motion_srvRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::cruiser_msgs::motion_srvRequest> should match 
// service_traits::DataType< ::cruiser_msgs::motion_srv > 
template<>
struct DataType< ::cruiser_msgs::motion_srvRequest>
{
  static const char* value()
  {
    return DataType< ::cruiser_msgs::motion_srv >::value();
  }
  static const char* value(const ::cruiser_msgs::motion_srvRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::cruiser_msgs::motion_srvResponse> should match 
// service_traits::MD5Sum< ::cruiser_msgs::motion_srv > 
template<>
struct MD5Sum< ::cruiser_msgs::motion_srvResponse>
{
  static const char* value()
  {
    return MD5Sum< ::cruiser_msgs::motion_srv >::value();
  }
  static const char* value(const ::cruiser_msgs::motion_srvResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::cruiser_msgs::motion_srvResponse> should match 
// service_traits::DataType< ::cruiser_msgs::motion_srv > 
template<>
struct DataType< ::cruiser_msgs::motion_srvResponse>
{
  static const char* value()
  {
    return DataType< ::cruiser_msgs::motion_srv >::value();
  }
  static const char* value(const ::cruiser_msgs::motion_srvResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // CRUISER_MSGS_MESSAGE_MOTION_SRV_H