// Generated by gencpp from file cav_project_2/limo_state.msg
// DO NOT EDIT!


#ifndef CAV_PROJECT_2_MESSAGE_LIMO_STATE_H
#define CAV_PROJECT_2_MESSAGE_LIMO_STATE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace cav_project_2
{
template <class ContainerAllocator>
struct limo_state_
{
  typedef limo_state_<ContainerAllocator> Type;

  limo_state_()
    : limoID()
    , vel(0.0)
    , d0(0.0)
    , d1(0.0)
    , v1(0.0)
    , d2(0.0)
    , v2(0.0)
    , vd(0.0)  {
    }
  limo_state_(const ContainerAllocator& _alloc)
    : limoID(_alloc)
    , vel(0.0)
    , d0(0.0)
    , d1(0.0)
    , v1(0.0)
    , d2(0.0)
    , v2(0.0)
    , vd(0.0)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _limoID_type;
  _limoID_type limoID;

   typedef double _vel_type;
  _vel_type vel;

   typedef double _d0_type;
  _d0_type d0;

   typedef double _d1_type;
  _d1_type d1;

   typedef double _v1_type;
  _v1_type v1;

   typedef double _d2_type;
  _d2_type d2;

   typedef double _v2_type;
  _v2_type v2;

   typedef double _vd_type;
  _vd_type vd;





  typedef boost::shared_ptr< ::cav_project_2::limo_state_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cav_project_2::limo_state_<ContainerAllocator> const> ConstPtr;

}; // struct limo_state_

typedef ::cav_project_2::limo_state_<std::allocator<void> > limo_state;

typedef boost::shared_ptr< ::cav_project_2::limo_state > limo_statePtr;
typedef boost::shared_ptr< ::cav_project_2::limo_state const> limo_stateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cav_project_2::limo_state_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cav_project_2::limo_state_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::cav_project_2::limo_state_<ContainerAllocator1> & lhs, const ::cav_project_2::limo_state_<ContainerAllocator2> & rhs)
{
  return lhs.limoID == rhs.limoID &&
    lhs.vel == rhs.vel &&
    lhs.d0 == rhs.d0 &&
    lhs.d1 == rhs.d1 &&
    lhs.v1 == rhs.v1 &&
    lhs.d2 == rhs.d2 &&
    lhs.v2 == rhs.v2 &&
    lhs.vd == rhs.vd;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::cav_project_2::limo_state_<ContainerAllocator1> & lhs, const ::cav_project_2::limo_state_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace cav_project_2

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::cav_project_2::limo_state_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cav_project_2::limo_state_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cav_project_2::limo_state_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cav_project_2::limo_state_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cav_project_2::limo_state_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cav_project_2::limo_state_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cav_project_2::limo_state_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a1fba4736fa7b1723499e2717d3f0429";
  }

  static const char* value(const ::cav_project_2::limo_state_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa1fba4736fa7b172ULL;
  static const uint64_t static_value2 = 0x3499e2717d3f0429ULL;
};

template<class ContainerAllocator>
struct DataType< ::cav_project_2::limo_state_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cav_project_2/limo_state";
  }

  static const char* value(const ::cav_project_2::limo_state_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cav_project_2::limo_state_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string limoID\n"
"float64 vel\n"
"float64 d0\n"
"float64 d1\n"
"float64 v1\n"
"float64 d2\n"
"float64 v2\n"
"float64 vd\n"
;
  }

  static const char* value(const ::cav_project_2::limo_state_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cav_project_2::limo_state_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.limoID);
      stream.next(m.vel);
      stream.next(m.d0);
      stream.next(m.d1);
      stream.next(m.v1);
      stream.next(m.d2);
      stream.next(m.v2);
      stream.next(m.vd);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct limo_state_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cav_project_2::limo_state_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cav_project_2::limo_state_<ContainerAllocator>& v)
  {
    s << indent << "limoID: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.limoID);
    s << indent << "vel: ";
    Printer<double>::stream(s, indent + "  ", v.vel);
    s << indent << "d0: ";
    Printer<double>::stream(s, indent + "  ", v.d0);
    s << indent << "d1: ";
    Printer<double>::stream(s, indent + "  ", v.d1);
    s << indent << "v1: ";
    Printer<double>::stream(s, indent + "  ", v.v1);
    s << indent << "d2: ";
    Printer<double>::stream(s, indent + "  ", v.d2);
    s << indent << "v2: ";
    Printer<double>::stream(s, indent + "  ", v.v2);
    s << indent << "vd: ";
    Printer<double>::stream(s, indent + "  ", v.vd);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CAV_PROJECT_2_MESSAGE_LIMO_STATE_H
