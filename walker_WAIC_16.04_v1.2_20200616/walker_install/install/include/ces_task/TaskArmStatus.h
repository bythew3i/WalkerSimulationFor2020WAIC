// Generated by gencpp from file ces_task/TaskArmStatus.msg
// DO NOT EDIT!


#ifndef CES_TASK_MESSAGE_TASKARMSTATUS_H
#define CES_TASK_MESSAGE_TASKARMSTATUS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace ces_task
{
template <class ContainerAllocator>
struct TaskArmStatus_
{
  typedef TaskArmStatus_<ContainerAllocator> Type;

  TaskArmStatus_()
    : header()
    , task_id()
    , demander()
    , executor()
    , task_status()
    , process_info()  {
    }
  TaskArmStatus_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , task_id(_alloc)
    , demander(_alloc)
    , executor(_alloc)
    , task_status(_alloc)
    , process_info(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _task_id_type;
  _task_id_type task_id;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _demander_type;
  _demander_type demander;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _executor_type;
  _executor_type executor;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _task_status_type;
  _task_status_type task_status;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _process_info_type;
  _process_info_type process_info;




  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  OWNER_NONE;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  OWNER_SDK;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  OWNER_LEGS;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  OWNER_ARMS;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  OWNER_REMOTER;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  OWNER_HOST;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  TASK_GREETINGS;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  TASK_HANDSHAKE;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  TASK_STANDBY;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  TASK_CHARGE;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  TASK_SWINGWHILEWALKE;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  TASK_WALKINHAND;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  TASK_DANCE;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  TASK_ENDPOINTCTRL;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  TASK_HYBIRDTRQCTRL;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  TASK_HYBIRDPOSCTRL;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  TASK_NULLSPACECTRL;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  TASK_VISUALSERVO;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  TASK_WHOLEBODYCTRL;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  TASK_DRAWING;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  TASK_SAYGOODBYE;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  TASK_OPENDOOR;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  TASK_GETBAG;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  TASK_HANGBAG;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  TASK_GETWATER;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  TASK_PLACEWATER;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  TASK_WALKWITHWATER;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  TASK_GETFOOD;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  TASK_WALKWITHFOOD;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  TASK_GIVEFOOD;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  TASK_RELEASEFOOD;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  TASK_GETUMBRELLA;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  TASK_GIVEUMBRELLA;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  TASK_RELEASEUMBRELLA;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  TASK_STATUS_IDLE;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  TASK_STATUS_BUSY;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  TASK_STATUS_STOP;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  TASK_STATUS_ABORT;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  TASK_STATUS_FAILED;

  typedef boost::shared_ptr< ::ces_task::TaskArmStatus_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ces_task::TaskArmStatus_<ContainerAllocator> const> ConstPtr;

}; // struct TaskArmStatus_

typedef ::ces_task::TaskArmStatus_<std::allocator<void> > TaskArmStatus;

typedef boost::shared_ptr< ::ces_task::TaskArmStatus > TaskArmStatusPtr;
typedef boost::shared_ptr< ::ces_task::TaskArmStatus const> TaskArmStatusConstPtr;

// constants requiring out of line definition

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TaskArmStatus_<ContainerAllocator>::OWNER_NONE =
        
          "none"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TaskArmStatus_<ContainerAllocator>::OWNER_SDK =
        
          "sdk"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TaskArmStatus_<ContainerAllocator>::OWNER_LEGS =
        
          "legs"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TaskArmStatus_<ContainerAllocator>::OWNER_ARMS =
        
          "arms"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TaskArmStatus_<ContainerAllocator>::OWNER_REMOTER =
        
          "remoter"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TaskArmStatus_<ContainerAllocator>::OWNER_HOST =
        
          "host"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TaskArmStatus_<ContainerAllocator>::TASK_GREETINGS =
        
          "CES/cesGreetings"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TaskArmStatus_<ContainerAllocator>::TASK_HANDSHAKE =
        
          "function/functionHybirdEffort"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TaskArmStatus_<ContainerAllocator>::TASK_STANDBY =
        
          "walk/walkDynamic"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TaskArmStatus_<ContainerAllocator>::TASK_CHARGE =
        
          "CES/cesRecharge"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TaskArmStatus_<ContainerAllocator>::TASK_SWINGWHILEWALKE =
        
          "walk/walkDynamic"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TaskArmStatus_<ContainerAllocator>::TASK_WALKINHAND =
        
          "walk/walkDynamicWithHand"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TaskArmStatus_<ContainerAllocator>::TASK_DANCE =
        
          "function/functionDance"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TaskArmStatus_<ContainerAllocator>::TASK_ENDPOINTCTRL =
        
          "function/functionEndpointControl"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TaskArmStatus_<ContainerAllocator>::TASK_HYBIRDTRQCTRL =
        
          "function/functionHybirdEffort"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TaskArmStatus_<ContainerAllocator>::TASK_HYBIRDPOSCTRL =
        
          "function/functionHybirdPosition"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TaskArmStatus_<ContainerAllocator>::TASK_NULLSPACECTRL =
        
          "function/functionNullspaceControl"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TaskArmStatus_<ContainerAllocator>::TASK_VISUALSERVO =
        
          "function/functionVisualServo"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TaskArmStatus_<ContainerAllocator>::TASK_WHOLEBODYCTRL =
        
          "function/functionWholeBodyControl"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TaskArmStatus_<ContainerAllocator>::TASK_DRAWING =
        
          "function/functionDrawing"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TaskArmStatus_<ContainerAllocator>::TASK_SAYGOODBYE =
        
          "CES/cesSayGoodbye"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TaskArmStatus_<ContainerAllocator>::TASK_OPENDOOR =
        
          "CES/ces100OpenDoor"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TaskArmStatus_<ContainerAllocator>::TASK_GETBAG =
        
          "CES/cesPickBag"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TaskArmStatus_<ContainerAllocator>::TASK_HANGBAG =
        
          "CES/ces101PlaceBag"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TaskArmStatus_<ContainerAllocator>::TASK_GETWATER =
        
          "CES/ces102OpenRefrigerator"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TaskArmStatus_<ContainerAllocator>::TASK_PLACEWATER =
        
          "CES/ces104PlaceWater"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TaskArmStatus_<ContainerAllocator>::TASK_WALKWITHWATER =
        
          "walk/walkDynamic"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TaskArmStatus_<ContainerAllocator>::TASK_GETFOOD =
        
          "CES/ces103PickChips"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TaskArmStatus_<ContainerAllocator>::TASK_WALKWITHFOOD =
        
          "walk/walkDynamic"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TaskArmStatus_<ContainerAllocator>::TASK_GIVEFOOD =
        
          "CES/cesDeliveryChips"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TaskArmStatus_<ContainerAllocator>::TASK_RELEASEFOOD =
        
          "CES/cesReleaseChips"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TaskArmStatus_<ContainerAllocator>::TASK_GETUMBRELLA =
        
          "CES/ces105PickUmbrella"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TaskArmStatus_<ContainerAllocator>::TASK_GIVEUMBRELLA =
        
          "CES/cesDeliveryUmbrella"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TaskArmStatus_<ContainerAllocator>::TASK_RELEASEUMBRELLA =
        
          "CES/cesReleaseUmbrella"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TaskArmStatus_<ContainerAllocator>::TASK_STATUS_IDLE =
        
          "idle"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TaskArmStatus_<ContainerAllocator>::TASK_STATUS_BUSY =
        
          "busy"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TaskArmStatus_<ContainerAllocator>::TASK_STATUS_STOP =
        
          "stopped"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TaskArmStatus_<ContainerAllocator>::TASK_STATUS_ABORT =
        
          "abort"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TaskArmStatus_<ContainerAllocator>::TASK_STATUS_FAILED =
        
          "fail"
        
        ;
   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ces_task::TaskArmStatus_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ces_task::TaskArmStatus_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ces_task

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'ces_task': ['/home/cjl/core_ws/walker_ws/walker2_motion_output/src/ces_task/msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ces_task::TaskArmStatus_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ces_task::TaskArmStatus_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ces_task::TaskArmStatus_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ces_task::TaskArmStatus_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ces_task::TaskArmStatus_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ces_task::TaskArmStatus_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ces_task::TaskArmStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e37a2b08a580018c6dc8764166a54283";
  }

  static const char* value(const ::ces_task::TaskArmStatus_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe37a2b08a580018cULL;
  static const uint64_t static_value2 = 0x6dc8764166a54283ULL;
};

template<class ContainerAllocator>
struct DataType< ::ces_task::TaskArmStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ces_task/TaskArmStatus";
  }

  static const char* value(const ::ces_task::TaskArmStatus_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ces_task::TaskArmStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# owner ids\n\
string OWNER_NONE=none\n\
string OWNER_SDK=sdk\n\
string OWNER_LEGS=legs\n\
string OWNER_ARMS=arms\n\
string OWNER_REMOTER=remoter\n\
string OWNER_HOST=host\n\
\n\
\n\
# task ids\n\
## common task\n\
string TASK_GREETINGS=CES/cesGreetings\n\
string TASK_HANDSHAKE=function/functionHybirdEffort\n\
string TASK_STANDBY=walk/walkDynamic\n\
string TASK_CHARGE=CES/cesRecharge\n\
string TASK_SWINGWHILEWALKE=walk/walkDynamic\n\
string TASK_WALKINHAND=walk/walkDynamicWithHand\n\
string TASK_DANCE=function/functionDance\n\
string TASK_ENDPOINTCTRL=function/functionEndpointControl\n\
string TASK_HYBIRDTRQCTRL=function/functionHybirdEffort\n\
string TASK_HYBIRDPOSCTRL=function/functionHybirdPosition\n\
string TASK_NULLSPACECTRL=function/functionNullspaceControl\n\
string TASK_VISUALSERVO=function/functionVisualServo\n\
string TASK_WHOLEBODYCTRL=function/functionWholeBodyControl\n\
string TASK_DRAWING=function/functionDrawing\n\
string TASK_SAYGOODBYE=CES/cesSayGoodbye\n\
\n\
## open door task\n\
string TASK_OPENDOOR=CES/ces100OpenDoor\n\
\n\
## handle bag task\n\
string TASK_GETBAG=CES/cesPickBag\n\
string TASK_HANGBAG=CES/ces101PlaceBag\n\
\n\
## get water task\n\
string TASK_GETWATER=CES/ces102OpenRefrigerator\n\
string TASK_PLACEWATER=CES/ces104PlaceWater\n\
string TASK_WALKWITHWATER=walk/walkDynamic\n\
\n\
## get food task\n\
string TASK_GETFOOD=CES/ces103PickChips\n\
string TASK_WALKWITHFOOD=walk/walkDynamic\n\
string TASK_GIVEFOOD=CES/cesDeliveryChips\n\
string TASK_RELEASEFOOD=CES/cesReleaseChips\n\
\n\
## get umbrella task\n\
string TASK_GETUMBRELLA=CES/ces105PickUmbrella\n\
string TASK_GIVEUMBRELLA=CES/cesDeliveryUmbrella\n\
string TASK_RELEASEUMBRELLA=CES/cesReleaseUmbrella\n\
\n\
\n\
# task status defines\n\
string TASK_STATUS_IDLE=idle\n\
string TASK_STATUS_BUSY=busy\n\
string TASK_STATUS_STOP=stopped\n\
string TASK_STATUS_ABORT=abort\n\
string TASK_STATUS_FAILED=fail\n\
\n\
##############################################\n\
\n\
# time stamp must be filled\n\
Header header\n\
\n\
# current task id\n\
string task_id\n\
\n\
# who send request\n\
string demander\n\
\n\
# who execute task\n\
string executor\n\
\n\
# current status\n\
string task_status\n\
\n\
# progress info(optional)\n\
string process_info\n\
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
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::ces_task::TaskArmStatus_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ces_task::TaskArmStatus_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.task_id);
      stream.next(m.demander);
      stream.next(m.executor);
      stream.next(m.task_status);
      stream.next(m.process_info);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TaskArmStatus_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ces_task::TaskArmStatus_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ces_task::TaskArmStatus_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "task_id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.task_id);
    s << indent << "demander: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.demander);
    s << indent << "executor: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.executor);
    s << indent << "task_status: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.task_status);
    s << indent << "process_info: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.process_info);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CES_TASK_MESSAGE_TASKARMSTATUS_H