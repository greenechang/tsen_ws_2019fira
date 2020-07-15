// Generated by gencpp from file op3_action_module_msgs/IsRunningResponse.msg
// DO NOT EDIT!


#ifndef OP3_ACTION_MODULE_MSGS_MESSAGE_ISRUNNINGRESPONSE_H
#define OP3_ACTION_MODULE_MSGS_MESSAGE_ISRUNNINGRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace op3_action_module_msgs
{
template <class ContainerAllocator>
struct IsRunningResponse_
{
  typedef IsRunningResponse_<ContainerAllocator> Type;

  IsRunningResponse_()
    : is_running(false)  {
    }
  IsRunningResponse_(const ContainerAllocator& _alloc)
    : is_running(false)  {
  (void)_alloc;
    }



   typedef uint8_t _is_running_type;
  _is_running_type is_running;





  typedef boost::shared_ptr< ::op3_action_module_msgs::IsRunningResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::op3_action_module_msgs::IsRunningResponse_<ContainerAllocator> const> ConstPtr;

}; // struct IsRunningResponse_

typedef ::op3_action_module_msgs::IsRunningResponse_<std::allocator<void> > IsRunningResponse;

typedef boost::shared_ptr< ::op3_action_module_msgs::IsRunningResponse > IsRunningResponsePtr;
typedef boost::shared_ptr< ::op3_action_module_msgs::IsRunningResponse const> IsRunningResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::op3_action_module_msgs::IsRunningResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::op3_action_module_msgs::IsRunningResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace op3_action_module_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'op3_action_module_msgs': ['/home/robotis/Tsen_ws/src/ROBOTIS-OP3-msgs/op3_action_module_msgs/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::op3_action_module_msgs::IsRunningResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::op3_action_module_msgs::IsRunningResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::op3_action_module_msgs::IsRunningResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::op3_action_module_msgs::IsRunningResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::op3_action_module_msgs::IsRunningResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::op3_action_module_msgs::IsRunningResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::op3_action_module_msgs::IsRunningResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ae3468a1af93d845e943210e7cef5a54";
  }

  static const char* value(const ::op3_action_module_msgs::IsRunningResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xae3468a1af93d845ULL;
  static const uint64_t static_value2 = 0xe943210e7cef5a54ULL;
};

template<class ContainerAllocator>
struct DataType< ::op3_action_module_msgs::IsRunningResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "op3_action_module_msgs/IsRunningResponse";
  }

  static const char* value(const ::op3_action_module_msgs::IsRunningResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::op3_action_module_msgs::IsRunningResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool is_running\n\
";
  }

  static const char* value(const ::op3_action_module_msgs::IsRunningResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::op3_action_module_msgs::IsRunningResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.is_running);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct IsRunningResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::op3_action_module_msgs::IsRunningResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::op3_action_module_msgs::IsRunningResponse_<ContainerAllocator>& v)
  {
    s << indent << "is_running: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.is_running);
  }
};

} // namespace message_operations
} // namespace ros

#endif // OP3_ACTION_MODULE_MSGS_MESSAGE_ISRUNNINGRESPONSE_H
