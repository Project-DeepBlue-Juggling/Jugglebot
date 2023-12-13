// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from jugglebot_interfaces:srv/GetRobotGeometry.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "jugglebot_interfaces/srv/detail/get_robot_geometry__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace jugglebot_interfaces
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void GetRobotGeometry_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) jugglebot_interfaces::srv::GetRobotGeometry_Request(_init);
}

void GetRobotGeometry_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<jugglebot_interfaces::srv::GetRobotGeometry_Request *>(message_memory);
  typed_message->~GetRobotGeometry_Request();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember GetRobotGeometry_Request_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jugglebot_interfaces::srv::GetRobotGeometry_Request, structure_needs_at_least_one_member),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers GetRobotGeometry_Request_message_members = {
  "jugglebot_interfaces::srv",  // message namespace
  "GetRobotGeometry_Request",  // message name
  1,  // number of fields
  sizeof(jugglebot_interfaces::srv::GetRobotGeometry_Request),
  GetRobotGeometry_Request_message_member_array,  // message members
  GetRobotGeometry_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  GetRobotGeometry_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t GetRobotGeometry_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GetRobotGeometry_Request_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace jugglebot_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<jugglebot_interfaces::srv::GetRobotGeometry_Request>()
{
  return &::jugglebot_interfaces::srv::rosidl_typesupport_introspection_cpp::GetRobotGeometry_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, jugglebot_interfaces, srv, GetRobotGeometry_Request)() {
  return &::jugglebot_interfaces::srv::rosidl_typesupport_introspection_cpp::GetRobotGeometry_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "jugglebot_interfaces/srv/detail/get_robot_geometry__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace jugglebot_interfaces
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void GetRobotGeometry_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) jugglebot_interfaces::srv::GetRobotGeometry_Response(_init);
}

void GetRobotGeometry_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<jugglebot_interfaces::srv::GetRobotGeometry_Response *>(message_memory);
  typed_message->~GetRobotGeometry_Response();
}

size_t size_function__GetRobotGeometry_Response__start_pos(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GetRobotGeometry_Response__start_pos(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__GetRobotGeometry_Response__start_pos(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void resize_function__GetRobotGeometry_Response__start_pos(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__GetRobotGeometry_Response__base_nodes(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GetRobotGeometry_Response__base_nodes(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__GetRobotGeometry_Response__base_nodes(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void resize_function__GetRobotGeometry_Response__base_nodes(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__GetRobotGeometry_Response__init_plat_nodes(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GetRobotGeometry_Response__init_plat_nodes(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__GetRobotGeometry_Response__init_plat_nodes(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void resize_function__GetRobotGeometry_Response__init_plat_nodes(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__GetRobotGeometry_Response__init_arm_nodes(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GetRobotGeometry_Response__init_arm_nodes(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__GetRobotGeometry_Response__init_arm_nodes(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void resize_function__GetRobotGeometry_Response__init_arm_nodes(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__GetRobotGeometry_Response__init_hand_nodes(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GetRobotGeometry_Response__init_hand_nodes(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__GetRobotGeometry_Response__init_hand_nodes(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void resize_function__GetRobotGeometry_Response__init_hand_nodes(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__GetRobotGeometry_Response__init_leg_lengths(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GetRobotGeometry_Response__init_leg_lengths(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__GetRobotGeometry_Response__init_leg_lengths(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void resize_function__GetRobotGeometry_Response__init_leg_lengths(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember GetRobotGeometry_Response_message_member_array[8] = {
  {
    "start_pos",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jugglebot_interfaces::srv::GetRobotGeometry_Response, start_pos),  // bytes offset in struct
    nullptr,  // default value
    size_function__GetRobotGeometry_Response__start_pos,  // size() function pointer
    get_const_function__GetRobotGeometry_Response__start_pos,  // get_const(index) function pointer
    get_function__GetRobotGeometry_Response__start_pos,  // get(index) function pointer
    resize_function__GetRobotGeometry_Response__start_pos  // resize(index) function pointer
  },
  {
    "base_nodes",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jugglebot_interfaces::srv::GetRobotGeometry_Response, base_nodes),  // bytes offset in struct
    nullptr,  // default value
    size_function__GetRobotGeometry_Response__base_nodes,  // size() function pointer
    get_const_function__GetRobotGeometry_Response__base_nodes,  // get_const(index) function pointer
    get_function__GetRobotGeometry_Response__base_nodes,  // get(index) function pointer
    resize_function__GetRobotGeometry_Response__base_nodes  // resize(index) function pointer
  },
  {
    "init_plat_nodes",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jugglebot_interfaces::srv::GetRobotGeometry_Response, init_plat_nodes),  // bytes offset in struct
    nullptr,  // default value
    size_function__GetRobotGeometry_Response__init_plat_nodes,  // size() function pointer
    get_const_function__GetRobotGeometry_Response__init_plat_nodes,  // get_const(index) function pointer
    get_function__GetRobotGeometry_Response__init_plat_nodes,  // get(index) function pointer
    resize_function__GetRobotGeometry_Response__init_plat_nodes  // resize(index) function pointer
  },
  {
    "init_arm_nodes",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jugglebot_interfaces::srv::GetRobotGeometry_Response, init_arm_nodes),  // bytes offset in struct
    nullptr,  // default value
    size_function__GetRobotGeometry_Response__init_arm_nodes,  // size() function pointer
    get_const_function__GetRobotGeometry_Response__init_arm_nodes,  // get_const(index) function pointer
    get_function__GetRobotGeometry_Response__init_arm_nodes,  // get(index) function pointer
    resize_function__GetRobotGeometry_Response__init_arm_nodes  // resize(index) function pointer
  },
  {
    "init_hand_nodes",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jugglebot_interfaces::srv::GetRobotGeometry_Response, init_hand_nodes),  // bytes offset in struct
    nullptr,  // default value
    size_function__GetRobotGeometry_Response__init_hand_nodes,  // size() function pointer
    get_const_function__GetRobotGeometry_Response__init_hand_nodes,  // get_const(index) function pointer
    get_function__GetRobotGeometry_Response__init_hand_nodes,  // get(index) function pointer
    resize_function__GetRobotGeometry_Response__init_hand_nodes  // resize(index) function pointer
  },
  {
    "init_leg_lengths",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jugglebot_interfaces::srv::GetRobotGeometry_Response, init_leg_lengths),  // bytes offset in struct
    nullptr,  // default value
    size_function__GetRobotGeometry_Response__init_leg_lengths,  // size() function pointer
    get_const_function__GetRobotGeometry_Response__init_leg_lengths,  // get_const(index) function pointer
    get_function__GetRobotGeometry_Response__init_leg_lengths,  // get(index) function pointer
    resize_function__GetRobotGeometry_Response__init_leg_lengths  // resize(index) function pointer
  },
  {
    "leg_stroke",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jugglebot_interfaces::srv::GetRobotGeometry_Response, leg_stroke),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "hand_stroke",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jugglebot_interfaces::srv::GetRobotGeometry_Response, hand_stroke),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers GetRobotGeometry_Response_message_members = {
  "jugglebot_interfaces::srv",  // message namespace
  "GetRobotGeometry_Response",  // message name
  8,  // number of fields
  sizeof(jugglebot_interfaces::srv::GetRobotGeometry_Response),
  GetRobotGeometry_Response_message_member_array,  // message members
  GetRobotGeometry_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  GetRobotGeometry_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t GetRobotGeometry_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GetRobotGeometry_Response_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace jugglebot_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<jugglebot_interfaces::srv::GetRobotGeometry_Response>()
{
  return &::jugglebot_interfaces::srv::rosidl_typesupport_introspection_cpp::GetRobotGeometry_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, jugglebot_interfaces, srv, GetRobotGeometry_Response)() {
  return &::jugglebot_interfaces::srv::rosidl_typesupport_introspection_cpp::GetRobotGeometry_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "jugglebot_interfaces/srv/detail/get_robot_geometry__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace jugglebot_interfaces
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers GetRobotGeometry_service_members = {
  "jugglebot_interfaces::srv",  // service namespace
  "GetRobotGeometry",  // service name
  // these two fields are initialized below on the first access
  // see get_service_type_support_handle<jugglebot_interfaces::srv::GetRobotGeometry>()
  nullptr,  // request message
  nullptr  // response message
};

static const rosidl_service_type_support_t GetRobotGeometry_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GetRobotGeometry_service_members,
  get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace jugglebot_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<jugglebot_interfaces::srv::GetRobotGeometry>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::jugglebot_interfaces::srv::rosidl_typesupport_introspection_cpp::GetRobotGeometry_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure that both the request_members_ and the response_members_ are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::jugglebot_interfaces::srv::GetRobotGeometry_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::jugglebot_interfaces::srv::GetRobotGeometry_Response
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, jugglebot_interfaces, srv, GetRobotGeometry)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<jugglebot_interfaces::srv::GetRobotGeometry>();
}

#ifdef __cplusplus
}
#endif
