// Generated by gencpp from file op3_online_walking_module_msgs/GetKinematicsPoseResponse.msg
// DO NOT EDIT!


#ifndef OP3_ONLINE_WALKING_MODULE_MSGS_MESSAGE_GETKINEMATICSPOSERESPONSE_H
#define OP3_ONLINE_WALKING_MODULE_MSGS_MESSAGE_GETKINEMATICSPOSERESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <op3_online_walking_module_msgs/KinematicsPose.h>

namespace op3_online_walking_module_msgs
{
template <class ContainerAllocator>
struct GetKinematicsPoseResponse_
{
  typedef GetKinematicsPoseResponse_<ContainerAllocator> Type;

  GetKinematicsPoseResponse_()
    : pose()  {
    }
  GetKinematicsPoseResponse_(const ContainerAllocator& _alloc)
    : pose(_alloc)  {
  (void)_alloc;
    }



   typedef  ::op3_online_walking_module_msgs::KinematicsPose_<ContainerAllocator>  _pose_type;
  _pose_type pose;





  typedef boost::shared_ptr< ::op3_online_walking_module_msgs::GetKinematicsPoseResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::op3_online_walking_module_msgs::GetKinematicsPoseResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GetKinematicsPoseResponse_

typedef ::op3_online_walking_module_msgs::GetKinematicsPoseResponse_<std::allocator<void> > GetKinematicsPoseResponse;

typedef boost::shared_ptr< ::op3_online_walking_module_msgs::GetKinematicsPoseResponse > GetKinematicsPoseResponsePtr;
typedef boost::shared_ptr< ::op3_online_walking_module_msgs::GetKinematicsPoseResponse const> GetKinematicsPoseResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::op3_online_walking_module_msgs::GetKinematicsPoseResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::op3_online_walking_module_msgs::GetKinematicsPoseResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace op3_online_walking_module_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'op3_online_walking_module_msgs': ['/home/robotis/Tsen_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg'], 'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::op3_online_walking_module_msgs::GetKinematicsPoseResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::op3_online_walking_module_msgs::GetKinematicsPoseResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::op3_online_walking_module_msgs::GetKinematicsPoseResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::op3_online_walking_module_msgs::GetKinematicsPoseResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::op3_online_walking_module_msgs::GetKinematicsPoseResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::op3_online_walking_module_msgs::GetKinematicsPoseResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::op3_online_walking_module_msgs::GetKinematicsPoseResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c75b426633d0bb0b22c3a048b9eac9ee";
  }

  static const char* value(const ::op3_online_walking_module_msgs::GetKinematicsPoseResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc75b426633d0bb0bULL;
  static const uint64_t static_value2 = 0x22c3a048b9eac9eeULL;
};

template<class ContainerAllocator>
struct DataType< ::op3_online_walking_module_msgs::GetKinematicsPoseResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "op3_online_walking_module_msgs/GetKinematicsPoseResponse";
  }

  static const char* value(const ::op3_online_walking_module_msgs::GetKinematicsPoseResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::op3_online_walking_module_msgs::GetKinematicsPoseResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "op3_online_walking_module_msgs/KinematicsPose pose\n\
\n\
\n\
================================================================================\n\
MSG: op3_online_walking_module_msgs/KinematicsPose\n\
string  name\n\
float64  mov_time\n\
geometry_msgs/Pose pose\n\
\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of position and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
";
  }

  static const char* value(const ::op3_online_walking_module_msgs::GetKinematicsPoseResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::op3_online_walking_module_msgs::GetKinematicsPoseResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.pose);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetKinematicsPoseResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::op3_online_walking_module_msgs::GetKinematicsPoseResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::op3_online_walking_module_msgs::GetKinematicsPoseResponse_<ContainerAllocator>& v)
  {
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::op3_online_walking_module_msgs::KinematicsPose_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
  }
};

} // namespace message_operations
} // namespace ros

#endif // OP3_ONLINE_WALKING_MODULE_MSGS_MESSAGE_GETKINEMATICSPOSERESPONSE_H
