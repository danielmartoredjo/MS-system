// Generated by gencpp from file hc_sr04/obj_sensor.msg
// DO NOT EDIT!


#ifndef HC_SR04_MESSAGE_OBJ_SENSOR_H
#define HC_SR04_MESSAGE_OBJ_SENSOR_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace hc_sr04
{
template <class ContainerAllocator>
struct obj_sensor_
{
  typedef obj_sensor_<ContainerAllocator> Type;

  obj_sensor_()
    : header()
    , sensor_id()
    , min_range(0.0)
    , max_range(0.0)
    , range(0.0)  {
    }
  obj_sensor_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , sensor_id(_alloc)
    , min_range(0.0)
    , max_range(0.0)
    , range(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _sensor_id_type;
  _sensor_id_type sensor_id;

   typedef float _min_range_type;
  _min_range_type min_range;

   typedef float _max_range_type;
  _max_range_type max_range;

   typedef float _range_type;
  _range_type range;




  typedef boost::shared_ptr< ::hc_sr04::obj_sensor_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hc_sr04::obj_sensor_<ContainerAllocator> const> ConstPtr;

}; // struct obj_sensor_

typedef ::hc_sr04::obj_sensor_<std::allocator<void> > obj_sensor;

typedef boost::shared_ptr< ::hc_sr04::obj_sensor > obj_sensorPtr;
typedef boost::shared_ptr< ::hc_sr04::obj_sensor const> obj_sensorConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hc_sr04::obj_sensor_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hc_sr04::obj_sensor_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace hc_sr04

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'hc_sr04': ['/home/pi/catkin_ws/src/ros-hc-sr04-node/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::hc_sr04::obj_sensor_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hc_sr04::obj_sensor_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hc_sr04::obj_sensor_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hc_sr04::obj_sensor_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hc_sr04::obj_sensor_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hc_sr04::obj_sensor_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hc_sr04::obj_sensor_<ContainerAllocator> >
{
  static const char* value()
  {
    return "128a332b8e44125de213da70967789cb";
  }

  static const char* value(const ::hc_sr04::obj_sensor_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x128a332b8e44125dULL;
  static const uint64_t static_value2 = 0xe213da70967789cbULL;
};

template<class ContainerAllocator>
struct DataType< ::hc_sr04::obj_sensor_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hc_sr04/obj_sensor";
  }

  static const char* value(const ::hc_sr04::obj_sensor_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hc_sr04::obj_sensor_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# This message also can represent a fixed-distance (binary) ranger.  This\n\
# sensor will have min_range===max_range===distance of detection.\n\
# These sensors follow REP 117 and will output -Inf if the object is detected\n\
# and +Inf if the object is outside of the detection range.\n\
\n\
Header header           # timestamp in the header is the time the ranger\n\
                        # returned the distance reading\n\
\n\
string sensor_id        # gives sensor id \n\
\n\
float32 min_range       # minimum range value [m]\n\
float32 max_range       # maximum range value [m]\n\
                        # Fixed distance rangers require min_range==max_range\n\
\n\
float32 range           # range data [m]\n\
                        # (Note: values < range_min or > range_max\n\
                        # should be discarded)\n\
                        # Fixed distance rangers only output -Inf or +Inf.\n\
                        # -Inf represents a detection within fixed distance.\n\
                        # (Detection too close to the sensor to quantify)\n\
                        # +Inf represents no detection within the fixed distance.\n\
                        # (Object out of range)\n\
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

  static const char* value(const ::hc_sr04::obj_sensor_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hc_sr04::obj_sensor_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.sensor_id);
      stream.next(m.min_range);
      stream.next(m.max_range);
      stream.next(m.range);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct obj_sensor_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hc_sr04::obj_sensor_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hc_sr04::obj_sensor_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "sensor_id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.sensor_id);
    s << indent << "min_range: ";
    Printer<float>::stream(s, indent + "  ", v.min_range);
    s << indent << "max_range: ";
    Printer<float>::stream(s, indent + "  ", v.max_range);
    s << indent << "range: ";
    Printer<float>::stream(s, indent + "  ", v.range);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HC_SR04_MESSAGE_OBJ_SENSOR_H