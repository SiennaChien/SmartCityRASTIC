// Generated by gencpp from file cav_project/QP_solution.msg
// DO NOT EDIT!


#ifndef CAV_PROJECT_MESSAGE_QP_SOLUTION_H
#define CAV_PROJECT_MESSAGE_QP_SOLUTION_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace cav_project
{
template <class ContainerAllocator>
struct QP_solution_
{
  typedef QP_solution_<ContainerAllocator> Type;

  QP_solution_()
    : u(0.0)  {
    }
  QP_solution_(const ContainerAllocator& _alloc)
    : u(0.0)  {
  (void)_alloc;
    }



   typedef double _u_type;
  _u_type u;





  typedef boost::shared_ptr< ::cav_project::QP_solution_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cav_project::QP_solution_<ContainerAllocator> const> ConstPtr;

}; // struct QP_solution_

typedef ::cav_project::QP_solution_<std::allocator<void> > QP_solution;

typedef boost::shared_ptr< ::cav_project::QP_solution > QP_solutionPtr;
typedef boost::shared_ptr< ::cav_project::QP_solution const> QP_solutionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cav_project::QP_solution_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cav_project::QP_solution_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::cav_project::QP_solution_<ContainerAllocator1> & lhs, const ::cav_project::QP_solution_<ContainerAllocator2> & rhs)
{
  return lhs.u == rhs.u;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::cav_project::QP_solution_<ContainerAllocator1> & lhs, const ::cav_project::QP_solution_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace cav_project

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::cav_project::QP_solution_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cav_project::QP_solution_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cav_project::QP_solution_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cav_project::QP_solution_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cav_project::QP_solution_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cav_project::QP_solution_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cav_project::QP_solution_<ContainerAllocator> >
{
  static const char* value()
  {
    return "988df341e727ad40b85d2b8acf9471e9";
  }

  static const char* value(const ::cav_project::QP_solution_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x988df341e727ad40ULL;
  static const uint64_t static_value2 = 0xb85d2b8acf9471e9ULL;
};

template<class ContainerAllocator>
struct DataType< ::cav_project::QP_solution_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cav_project/QP_solution";
  }

  static const char* value(const ::cav_project::QP_solution_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cav_project::QP_solution_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 u\n"
;
  }

  static const char* value(const ::cav_project::QP_solution_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cav_project::QP_solution_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.u);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct QP_solution_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cav_project::QP_solution_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cav_project::QP_solution_<ContainerAllocator>& v)
  {
    s << indent << "u: ";
    Printer<double>::stream(s, indent + "  ", v.u);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CAV_PROJECT_MESSAGE_QP_SOLUTION_H
