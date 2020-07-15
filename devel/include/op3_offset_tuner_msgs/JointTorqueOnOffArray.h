// Generated by gencpp from file op3_offset_tuner_msgs/JointTorqueOnOffArray.msg
// DO NOT EDIT!


#ifndef OP3_OFFSET_TUNER_MSGS_MESSAGE_JOINTTORQUEONOFFARRAY_H
#define OP3_OFFSET_TUNER_MSGS_MESSAGE_JOINTTORQUEONOFFARRAY_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <op3_offset_tuner_msgs/JointTorqueOnOff.h>

namespace op3_offset_tuner_msgs
{
template <class ContainerAllocator>
struct JointTorqueOnOffArray_
{
  typedef JointTorqueOnOffArray_<ContainerAllocator> Type;

  JointTorqueOnOffArray_()
    : torque_enable_data()  {
    }
  JointTorqueOnOffArray_(const ContainerAllocator& _alloc)
    : torque_enable_data(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::op3_offset_tuner_msgs::JointTorqueOnOff_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::op3_offset_tuner_msgs::JointTorqueOnOff_<ContainerAllocator> >::other >  _torque_enable_data_type;
  _torque_enable_data_type torque_enable_data;





  typedef boost::shared_ptr< ::op3_offset_tuner_msgs::JointTorqueOnOffArray_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::op3_offset_tuner_msgs::JointTorqueOnOffArray_<ContainerAllocator> const> ConstPtr;

}; // struct JointTorqueOnOffArray_

typedef ::op3_offset_tuner_msgs::JointTorqueOnOffArray_<std::allocator<void> > JointTorqueOnOffArray;

typedef boost::shared_ptr< ::op3_offset_tuner_msgs::JointTorqueOnOffArray > JointTorqueOnOffArrayPtr;
typedef boost::shared_ptr< ::op3_offset_tuner_msgs::JointTorqueOnOffArray const> JointTorqueOnOffArrayConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::op3_offset_tuner_msgs::JointTorqueOnOffArray_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::op3_offset_tuner_msgs::JointTorqueOnOffArray_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace op3_offset_tuner_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'op3_offset_tuner_msgs': ['/home/robotis/Tsen_ws/src/ROBOTIS-OP3-msgs/op3_offset_tuner_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::op3_offset_tuner_msgs::JointTorqueOnOffArray_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::op3_offset_tuner_msgs::JointTorqueOnOffArray_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::op3_offset_tuner_msgs::JointTorqueOnOffArray_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::op3_offset_tuner_msgs::JointTorqueOnOffArray_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::op3_offset_tuner_msgs::JointTorqueOnOffArray_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::op3_offset_tuner_msgs::JointTorqueOnOffArray_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::op3_offset_tuner_msgs::JointTorqueOnOffArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1ca4db772b4d802ac00aebf4469fc8bf";
  }

  static const char* value(const ::op3_offset_tuner_msgs::JointTorqueOnOffArray_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1ca4db772b4d802aULL;
  static const uint64_t static_value2 = 0xc00aebf4469fc8bfULL;
};

template<class ContainerAllocator>
struct DataType< ::op3_offset_tuner_msgs::JointTorqueOnOffArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "op3_offset_tuner_msgs/JointTorqueOnOffArray";
  }

  static const char* value(const ::op3_offset_tuner_msgs::JointTorqueOnOffArray_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::op3_offset_tuner_msgs::JointTorqueOnOffArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "JointTorqueOnOff[] torque_enable_data\n\
================================================================================\n\
MSG: op3_offset_tuner_msgs/JointTorqueOnOff\n\
string  joint_name\n\
bool    torque_enable\n\
";
  }

  static const char* value(const ::op3_offset_tuner_msgs::JointTorqueOnOffArray_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::op3_offset_tuner_msgs::JointTorqueOnOffArray_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.torque_enable_data);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct JointTorqueOnOffArray_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::op3_offset_tuner_msgs::JointTorqueOnOffArray_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::op3_offset_tuner_msgs::JointTorqueOnOffArray_<ContainerAllocator>& v)
  {
    s << indent << "torque_enable_data[]" << std::endl;
    for (size_t i = 0; i < v.torque_enable_data.size(); ++i)
    {
      s << indent << "  torque_enable_data[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::op3_offset_tuner_msgs::JointTorqueOnOff_<ContainerAllocator> >::stream(s, indent + "    ", v.torque_enable_data[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // OP3_OFFSET_TUNER_MSGS_MESSAGE_JOINTTORQUEONOFFARRAY_H
