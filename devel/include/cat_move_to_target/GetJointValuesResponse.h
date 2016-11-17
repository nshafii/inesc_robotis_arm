// Generated by gencpp from file cat_move_to_target/GetJointValuesResponse.msg
// DO NOT EDIT!


#ifndef CAT_MOVE_TO_TARGET_MESSAGE_GETJOINTVALUESRESPONSE_H
#define CAT_MOVE_TO_TARGET_MESSAGE_GETJOINTVALUESRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace cat_move_to_target
{
template <class ContainerAllocator>
struct GetJointValuesResponse_
{
  typedef GetJointValuesResponse_<ContainerAllocator> Type;

  GetJointValuesResponse_()
    : theta1(0.0)
    , theta2(0.0)
    , theta3(0.0)
    , theta4(0.0)
    , theta5(0.0)
    , theta6(0.0)  {
    }
  GetJointValuesResponse_(const ContainerAllocator& _alloc)
    : theta1(0.0)
    , theta2(0.0)
    , theta3(0.0)
    , theta4(0.0)
    , theta5(0.0)
    , theta6(0.0)  {
  (void)_alloc;
    }



   typedef double _theta1_type;
  _theta1_type theta1;

   typedef double _theta2_type;
  _theta2_type theta2;

   typedef double _theta3_type;
  _theta3_type theta3;

   typedef double _theta4_type;
  _theta4_type theta4;

   typedef double _theta5_type;
  _theta5_type theta5;

   typedef double _theta6_type;
  _theta6_type theta6;




  typedef boost::shared_ptr< ::cat_move_to_target::GetJointValuesResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cat_move_to_target::GetJointValuesResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GetJointValuesResponse_

typedef ::cat_move_to_target::GetJointValuesResponse_<std::allocator<void> > GetJointValuesResponse;

typedef boost::shared_ptr< ::cat_move_to_target::GetJointValuesResponse > GetJointValuesResponsePtr;
typedef boost::shared_ptr< ::cat_move_to_target::GetJointValuesResponse const> GetJointValuesResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cat_move_to_target::GetJointValuesResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cat_move_to_target::GetJointValuesResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace cat_move_to_target

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::cat_move_to_target::GetJointValuesResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cat_move_to_target::GetJointValuesResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cat_move_to_target::GetJointValuesResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cat_move_to_target::GetJointValuesResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cat_move_to_target::GetJointValuesResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cat_move_to_target::GetJointValuesResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cat_move_to_target::GetJointValuesResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "01c16b8753716d52de66552164fe680a";
  }

  static const char* value(const ::cat_move_to_target::GetJointValuesResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x01c16b8753716d52ULL;
  static const uint64_t static_value2 = 0xde66552164fe680aULL;
};

template<class ContainerAllocator>
struct DataType< ::cat_move_to_target::GetJointValuesResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cat_move_to_target/GetJointValuesResponse";
  }

  static const char* value(const ::cat_move_to_target::GetJointValuesResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cat_move_to_target::GetJointValuesResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 theta1\n\
float64 theta2\n\
float64 theta3\n\
float64 theta4\n\
float64 theta5\n\
float64 theta6\n\
\n\
";
  }

  static const char* value(const ::cat_move_to_target::GetJointValuesResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cat_move_to_target::GetJointValuesResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.theta1);
      stream.next(m.theta2);
      stream.next(m.theta3);
      stream.next(m.theta4);
      stream.next(m.theta5);
      stream.next(m.theta6);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct GetJointValuesResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cat_move_to_target::GetJointValuesResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cat_move_to_target::GetJointValuesResponse_<ContainerAllocator>& v)
  {
    s << indent << "theta1: ";
    Printer<double>::stream(s, indent + "  ", v.theta1);
    s << indent << "theta2: ";
    Printer<double>::stream(s, indent + "  ", v.theta2);
    s << indent << "theta3: ";
    Printer<double>::stream(s, indent + "  ", v.theta3);
    s << indent << "theta4: ";
    Printer<double>::stream(s, indent + "  ", v.theta4);
    s << indent << "theta5: ";
    Printer<double>::stream(s, indent + "  ", v.theta5);
    s << indent << "theta6: ";
    Printer<double>::stream(s, indent + "  ", v.theta6);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CAT_MOVE_TO_TARGET_MESSAGE_GETJOINTVALUESRESPONSE_H
