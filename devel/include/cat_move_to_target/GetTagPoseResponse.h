// Generated by gencpp from file cat_move_to_target/GetTagPoseResponse.msg
// DO NOT EDIT!


#ifndef CAT_MOVE_TO_TARGET_MESSAGE_GETTAGPOSERESPONSE_H
#define CAT_MOVE_TO_TARGET_MESSAGE_GETTAGPOSERESPONSE_H


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
struct GetTagPoseResponse_
{
  typedef GetTagPoseResponse_<ContainerAllocator> Type;

  GetTagPoseResponse_()
    : pos_x(0.0)
    , pos_y(0.0)
    , pos_z(0.0)
    , ori_x(0.0)
    , ori_y(0.0)
    , ori_z(0.0)
    , ori_w(0.0)
    , w_pos_x(0.0)
    , w_pos_y(0.0)
    , w_pos_z(0.0)
    , foundtag(false)
    , tag_id(0)  {
    }
  GetTagPoseResponse_(const ContainerAllocator& _alloc)
    : pos_x(0.0)
    , pos_y(0.0)
    , pos_z(0.0)
    , ori_x(0.0)
    , ori_y(0.0)
    , ori_z(0.0)
    , ori_w(0.0)
    , w_pos_x(0.0)
    , w_pos_y(0.0)
    , w_pos_z(0.0)
    , foundtag(false)
    , tag_id(0)  {
  (void)_alloc;
    }



   typedef double _pos_x_type;
  _pos_x_type pos_x;

   typedef double _pos_y_type;
  _pos_y_type pos_y;

   typedef double _pos_z_type;
  _pos_z_type pos_z;

   typedef double _ori_x_type;
  _ori_x_type ori_x;

   typedef double _ori_y_type;
  _ori_y_type ori_y;

   typedef double _ori_z_type;
  _ori_z_type ori_z;

   typedef double _ori_w_type;
  _ori_w_type ori_w;

   typedef double _w_pos_x_type;
  _w_pos_x_type w_pos_x;

   typedef double _w_pos_y_type;
  _w_pos_y_type w_pos_y;

   typedef double _w_pos_z_type;
  _w_pos_z_type w_pos_z;

   typedef uint8_t _foundtag_type;
  _foundtag_type foundtag;

   typedef int16_t _tag_id_type;
  _tag_id_type tag_id;




  typedef boost::shared_ptr< ::cat_move_to_target::GetTagPoseResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cat_move_to_target::GetTagPoseResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GetTagPoseResponse_

typedef ::cat_move_to_target::GetTagPoseResponse_<std::allocator<void> > GetTagPoseResponse;

typedef boost::shared_ptr< ::cat_move_to_target::GetTagPoseResponse > GetTagPoseResponsePtr;
typedef boost::shared_ptr< ::cat_move_to_target::GetTagPoseResponse const> GetTagPoseResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cat_move_to_target::GetTagPoseResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cat_move_to_target::GetTagPoseResponse_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::cat_move_to_target::GetTagPoseResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cat_move_to_target::GetTagPoseResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cat_move_to_target::GetTagPoseResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cat_move_to_target::GetTagPoseResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cat_move_to_target::GetTagPoseResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cat_move_to_target::GetTagPoseResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cat_move_to_target::GetTagPoseResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f0edf0bfaf19feb00a1c38682a7a58d5";
  }

  static const char* value(const ::cat_move_to_target::GetTagPoseResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf0edf0bfaf19feb0ULL;
  static const uint64_t static_value2 = 0x0a1c38682a7a58d5ULL;
};

template<class ContainerAllocator>
struct DataType< ::cat_move_to_target::GetTagPoseResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cat_move_to_target/GetTagPoseResponse";
  }

  static const char* value(const ::cat_move_to_target::GetTagPoseResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cat_move_to_target::GetTagPoseResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 pos_x\n\
float64 pos_y\n\
float64 pos_z\n\
float64 ori_x\n\
float64 ori_y\n\
float64 ori_z\n\
float64 ori_w\n\
float64 w_pos_x\n\
float64 w_pos_y\n\
float64 w_pos_z\n\
bool foundtag\n\
int16 tag_id\n\
\n\
\n\
";
  }

  static const char* value(const ::cat_move_to_target::GetTagPoseResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cat_move_to_target::GetTagPoseResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.pos_x);
      stream.next(m.pos_y);
      stream.next(m.pos_z);
      stream.next(m.ori_x);
      stream.next(m.ori_y);
      stream.next(m.ori_z);
      stream.next(m.ori_w);
      stream.next(m.w_pos_x);
      stream.next(m.w_pos_y);
      stream.next(m.w_pos_z);
      stream.next(m.foundtag);
      stream.next(m.tag_id);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct GetTagPoseResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cat_move_to_target::GetTagPoseResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cat_move_to_target::GetTagPoseResponse_<ContainerAllocator>& v)
  {
    s << indent << "pos_x: ";
    Printer<double>::stream(s, indent + "  ", v.pos_x);
    s << indent << "pos_y: ";
    Printer<double>::stream(s, indent + "  ", v.pos_y);
    s << indent << "pos_z: ";
    Printer<double>::stream(s, indent + "  ", v.pos_z);
    s << indent << "ori_x: ";
    Printer<double>::stream(s, indent + "  ", v.ori_x);
    s << indent << "ori_y: ";
    Printer<double>::stream(s, indent + "  ", v.ori_y);
    s << indent << "ori_z: ";
    Printer<double>::stream(s, indent + "  ", v.ori_z);
    s << indent << "ori_w: ";
    Printer<double>::stream(s, indent + "  ", v.ori_w);
    s << indent << "w_pos_x: ";
    Printer<double>::stream(s, indent + "  ", v.w_pos_x);
    s << indent << "w_pos_y: ";
    Printer<double>::stream(s, indent + "  ", v.w_pos_y);
    s << indent << "w_pos_z: ";
    Printer<double>::stream(s, indent + "  ", v.w_pos_z);
    s << indent << "foundtag: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.foundtag);
    s << indent << "tag_id: ";
    Printer<int16_t>::stream(s, indent + "  ", v.tag_id);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CAT_MOVE_TO_TARGET_MESSAGE_GETTAGPOSERESPONSE_H
