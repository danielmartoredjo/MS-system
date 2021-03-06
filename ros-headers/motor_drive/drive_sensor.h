// Generated by gencpp from file motor_drive/drive_sensor.msg
// DO NOT EDIT!


#ifndef MOTOR_DRIVE_MESSAGE_DRIVE_SENSOR_H
#define MOTOR_DRIVE_MESSAGE_DRIVE_SENSOR_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace motor_drive
{
template <class ContainerAllocator>
struct drive_sensor_
{
  typedef drive_sensor_<ContainerAllocator> Type;

  drive_sensor_()
    : header()
    , sensor_id()
    , pwinit(0.0)
    , pw(0.0)  {
    }
  drive_sensor_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , sensor_id(_alloc)
    , pwinit(0.0)
    , pw(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _sensor_id_type;
  _sensor_id_type sensor_id;

   typedef float _pwinit_type;
  _pwinit_type pwinit;

   typedef float _pw_type;
  _pw_type pw;




  typedef boost::shared_ptr< ::motor_drive::drive_sensor_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::motor_drive::drive_sensor_<ContainerAllocator> const> ConstPtr;

}; // struct drive_sensor_

typedef ::motor_drive::drive_sensor_<std::allocator<void> > drive_sensor;

typedef boost::shared_ptr< ::motor_drive::drive_sensor > drive_sensorPtr;
typedef boost::shared_ptr< ::motor_drive::drive_sensor const> drive_sensorConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::motor_drive::drive_sensor_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::motor_drive::drive_sensor_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace motor_drive

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'motor_drive': ['/home/pi/catkin_ws/src/ros-motor-node/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::motor_drive::drive_sensor_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::motor_drive::drive_sensor_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::motor_drive::drive_sensor_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::motor_drive::drive_sensor_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::motor_drive::drive_sensor_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::motor_drive::drive_sensor_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::motor_drive::drive_sensor_<ContainerAllocator> >
{
  static const char* value()
  {
    return "57d62d2c596d578891e30e31bb9d52de";
  }

  static const char* value(const ::motor_drive::drive_sensor_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x57d62d2c596d5788ULL;
  static const uint64_t static_value2 = 0x91e30e31bb9d52deULL;
};

template<class ContainerAllocator>
struct DataType< ::motor_drive::drive_sensor_<ContainerAllocator> >
{
  static const char* value()
  {
    return "motor_drive/drive_sensor";
  }

  static const char* value(const ::motor_drive::drive_sensor_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::motor_drive::drive_sensor_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header           # timestamp in the header is the time the ranger\n\
                        # returned the distance reading\n\
\n\
string sensor_id        # gives sensor id \n\
\n\
float32 pwinit # value for calibration\n\
\n\
float32 pw     # value the pulse width is set too\n\
\n\
\n\
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

  static const char* value(const ::motor_drive::drive_sensor_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::motor_drive::drive_sensor_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.sensor_id);
      stream.next(m.pwinit);
      stream.next(m.pw);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct drive_sensor_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::motor_drive::drive_sensor_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::motor_drive::drive_sensor_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "sensor_id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.sensor_id);
    s << indent << "pwinit: ";
    Printer<float>::stream(s, indent + "  ", v.pwinit);
    s << indent << "pw: ";
    Printer<float>::stream(s, indent + "  ", v.pw);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MOTOR_DRIVE_MESSAGE_DRIVE_SENSOR_H
