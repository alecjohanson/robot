/* Auto-generated by genmsg_cpp for file /home/kth/Documents/DD2425_2013/fuerte_workspace/rosserial/rosserial_msgs/srv/RequestParam.srv */
#ifndef ROSSERIAL_MSGS_SERVICE_REQUESTPARAM_H
#define ROSSERIAL_MSGS_SERVICE_REQUESTPARAM_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "ros/service_traits.h"




namespace rosserial_msgs
{
template <class ContainerAllocator>
struct RequestParamRequest_ {
  typedef RequestParamRequest_<ContainerAllocator> Type;

  RequestParamRequest_()
  : name()
  {
  }

  RequestParamRequest_(const ContainerAllocator& _alloc)
  : name(_alloc)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _name_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  name;


  typedef boost::shared_ptr< ::rosserial_msgs::RequestParamRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rosserial_msgs::RequestParamRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct RequestParamRequest
typedef  ::rosserial_msgs::RequestParamRequest_<std::allocator<void> > RequestParamRequest;

typedef boost::shared_ptr< ::rosserial_msgs::RequestParamRequest> RequestParamRequestPtr;
typedef boost::shared_ptr< ::rosserial_msgs::RequestParamRequest const> RequestParamRequestConstPtr;


template <class ContainerAllocator>
struct RequestParamResponse_ {
  typedef RequestParamResponse_<ContainerAllocator> Type;

  RequestParamResponse_()
  : ints()
  , floats()
  , strings()
  {
  }

  RequestParamResponse_(const ContainerAllocator& _alloc)
  : ints(_alloc)
  , floats(_alloc)
  , strings(_alloc)
  {
  }

  typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _ints_type;
  std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  ints;

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _floats_type;
  std::vector<float, typename ContainerAllocator::template rebind<float>::other >  floats;

  typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _strings_type;
  std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  strings;


  typedef boost::shared_ptr< ::rosserial_msgs::RequestParamResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rosserial_msgs::RequestParamResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct RequestParamResponse
typedef  ::rosserial_msgs::RequestParamResponse_<std::allocator<void> > RequestParamResponse;

typedef boost::shared_ptr< ::rosserial_msgs::RequestParamResponse> RequestParamResponsePtr;
typedef boost::shared_ptr< ::rosserial_msgs::RequestParamResponse const> RequestParamResponseConstPtr;

struct RequestParam
{

typedef RequestParamRequest Request;
typedef RequestParamResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct RequestParam
} // namespace rosserial_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::rosserial_msgs::RequestParamRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::rosserial_msgs::RequestParamRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::rosserial_msgs::RequestParamRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "c1f3d28f1b044c871e6eff2e9fc3c667";
  }

  static const char* value(const  ::rosserial_msgs::RequestParamRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xc1f3d28f1b044c87ULL;
  static const uint64_t static_value2 = 0x1e6eff2e9fc3c667ULL;
};

template<class ContainerAllocator>
struct DataType< ::rosserial_msgs::RequestParamRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "rosserial_msgs/RequestParamRequest";
  }

  static const char* value(const  ::rosserial_msgs::RequestParamRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::rosserial_msgs::RequestParamRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "string name\n\
\n\
\n\
";
  }

  static const char* value(const  ::rosserial_msgs::RequestParamRequest_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::rosserial_msgs::RequestParamResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::rosserial_msgs::RequestParamResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::rosserial_msgs::RequestParamResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "9f0e98bda65981986ddf53afa7a40e49";
  }

  static const char* value(const  ::rosserial_msgs::RequestParamResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x9f0e98bda6598198ULL;
  static const uint64_t static_value2 = 0x6ddf53afa7a40e49ULL;
};

template<class ContainerAllocator>
struct DataType< ::rosserial_msgs::RequestParamResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "rosserial_msgs/RequestParamResponse";
  }

  static const char* value(const  ::rosserial_msgs::RequestParamResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::rosserial_msgs::RequestParamResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
int32[]   ints\n\
float32[] floats\n\
string[]  strings\n\
\n\
\n\
";
  }

  static const char* value(const  ::rosserial_msgs::RequestParamResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::rosserial_msgs::RequestParamRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.name);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct RequestParamRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::rosserial_msgs::RequestParamResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.ints);
    stream.next(m.floats);
    stream.next(m.strings);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct RequestParamResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<rosserial_msgs::RequestParam> {
  static const char* value() 
  {
    return "d7a0c2be00c9fd03cc69f2863de9c4d9";
  }

  static const char* value(const rosserial_msgs::RequestParam&) { return value(); } 
};

template<>
struct DataType<rosserial_msgs::RequestParam> {
  static const char* value() 
  {
    return "rosserial_msgs/RequestParam";
  }

  static const char* value(const rosserial_msgs::RequestParam&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<rosserial_msgs::RequestParamRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d7a0c2be00c9fd03cc69f2863de9c4d9";
  }

  static const char* value(const rosserial_msgs::RequestParamRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<rosserial_msgs::RequestParamRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "rosserial_msgs/RequestParam";
  }

  static const char* value(const rosserial_msgs::RequestParamRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<rosserial_msgs::RequestParamResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d7a0c2be00c9fd03cc69f2863de9c4d9";
  }

  static const char* value(const rosserial_msgs::RequestParamResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<rosserial_msgs::RequestParamResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "rosserial_msgs/RequestParam";
  }

  static const char* value(const rosserial_msgs::RequestParamResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // ROSSERIAL_MSGS_SERVICE_REQUESTPARAM_H

