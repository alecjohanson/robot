/* Auto-generated by genmsg_cpp for file /home/robo/ros_workspace/robot/rosserial/rosserial_arduino/srv/Test.srv */
#ifndef ROSSERIAL_ARDUINO_SERVICE_TEST_H
#define ROSSERIAL_ARDUINO_SERVICE_TEST_H
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




namespace rosserial_arduino
{
template <class ContainerAllocator>
struct TestRequest_ {
  typedef TestRequest_<ContainerAllocator> Type;

  TestRequest_()
  : input()
  {
  }

  TestRequest_(const ContainerAllocator& _alloc)
  : input(_alloc)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _input_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  input;


  typedef boost::shared_ptr< ::rosserial_arduino::TestRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rosserial_arduino::TestRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct TestRequest
typedef  ::rosserial_arduino::TestRequest_<std::allocator<void> > TestRequest;

typedef boost::shared_ptr< ::rosserial_arduino::TestRequest> TestRequestPtr;
typedef boost::shared_ptr< ::rosserial_arduino::TestRequest const> TestRequestConstPtr;


template <class ContainerAllocator>
struct TestResponse_ {
  typedef TestResponse_<ContainerAllocator> Type;

  TestResponse_()
  : output()
  {
  }

  TestResponse_(const ContainerAllocator& _alloc)
  : output(_alloc)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _output_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  output;


  typedef boost::shared_ptr< ::rosserial_arduino::TestResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rosserial_arduino::TestResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct TestResponse
typedef  ::rosserial_arduino::TestResponse_<std::allocator<void> > TestResponse;

typedef boost::shared_ptr< ::rosserial_arduino::TestResponse> TestResponsePtr;
typedef boost::shared_ptr< ::rosserial_arduino::TestResponse const> TestResponseConstPtr;

struct Test
{

typedef TestRequest Request;
typedef TestResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct Test
} // namespace rosserial_arduino

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::rosserial_arduino::TestRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::rosserial_arduino::TestRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::rosserial_arduino::TestRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "39e92f1778057359c64c7b8a7d7b19de";
  }

  static const char* value(const  ::rosserial_arduino::TestRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x39e92f1778057359ULL;
  static const uint64_t static_value2 = 0xc64c7b8a7d7b19deULL;
};

template<class ContainerAllocator>
struct DataType< ::rosserial_arduino::TestRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "rosserial_arduino/TestRequest";
  }

  static const char* value(const  ::rosserial_arduino::TestRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::rosserial_arduino::TestRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "string input\n\
\n\
";
  }

  static const char* value(const  ::rosserial_arduino::TestRequest_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::rosserial_arduino::TestResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::rosserial_arduino::TestResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::rosserial_arduino::TestResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "0825d95fdfa2c8f4bbb4e9c74bccd3fd";
  }

  static const char* value(const  ::rosserial_arduino::TestResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x0825d95fdfa2c8f4ULL;
  static const uint64_t static_value2 = 0xbbb4e9c74bccd3fdULL;
};

template<class ContainerAllocator>
struct DataType< ::rosserial_arduino::TestResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "rosserial_arduino/TestResponse";
  }

  static const char* value(const  ::rosserial_arduino::TestResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::rosserial_arduino::TestResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "string output\n\
\n\
\n\
";
  }

  static const char* value(const  ::rosserial_arduino::TestResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::rosserial_arduino::TestRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.input);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct TestRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::rosserial_arduino::TestResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.output);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct TestResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<rosserial_arduino::Test> {
  static const char* value() 
  {
    return "c63e85f503b805d84df783e71c6bb0d2";
  }

  static const char* value(const rosserial_arduino::Test&) { return value(); } 
};

template<>
struct DataType<rosserial_arduino::Test> {
  static const char* value() 
  {
    return "rosserial_arduino/Test";
  }

  static const char* value(const rosserial_arduino::Test&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<rosserial_arduino::TestRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "c63e85f503b805d84df783e71c6bb0d2";
  }

  static const char* value(const rosserial_arduino::TestRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<rosserial_arduino::TestRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "rosserial_arduino/Test";
  }

  static const char* value(const rosserial_arduino::TestRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<rosserial_arduino::TestResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "c63e85f503b805d84df783e71c6bb0d2";
  }

  static const char* value(const rosserial_arduino::TestResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<rosserial_arduino::TestResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "rosserial_arduino/Test";
  }

  static const char* value(const rosserial_arduino::TestResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // ROSSERIAL_ARDUINO_SERVICE_TEST_H

