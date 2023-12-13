// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from jugglebot_interfaces:srv/GetRobotGeometry.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "jugglebot_interfaces/srv/detail/get_robot_geometry__rosidl_typesupport_introspection_c.h"
#include "jugglebot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "jugglebot_interfaces/srv/detail/get_robot_geometry__functions.h"
#include "jugglebot_interfaces/srv/detail/get_robot_geometry__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void GetRobotGeometry_Request__rosidl_typesupport_introspection_c__GetRobotGeometry_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  jugglebot_interfaces__srv__GetRobotGeometry_Request__init(message_memory);
}

void GetRobotGeometry_Request__rosidl_typesupport_introspection_c__GetRobotGeometry_Request_fini_function(void * message_memory)
{
  jugglebot_interfaces__srv__GetRobotGeometry_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember GetRobotGeometry_Request__rosidl_typesupport_introspection_c__GetRobotGeometry_Request_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jugglebot_interfaces__srv__GetRobotGeometry_Request, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers GetRobotGeometry_Request__rosidl_typesupport_introspection_c__GetRobotGeometry_Request_message_members = {
  "jugglebot_interfaces__srv",  // message namespace
  "GetRobotGeometry_Request",  // message name
  1,  // number of fields
  sizeof(jugglebot_interfaces__srv__GetRobotGeometry_Request),
  GetRobotGeometry_Request__rosidl_typesupport_introspection_c__GetRobotGeometry_Request_message_member_array,  // message members
  GetRobotGeometry_Request__rosidl_typesupport_introspection_c__GetRobotGeometry_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  GetRobotGeometry_Request__rosidl_typesupport_introspection_c__GetRobotGeometry_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t GetRobotGeometry_Request__rosidl_typesupport_introspection_c__GetRobotGeometry_Request_message_type_support_handle = {
  0,
  &GetRobotGeometry_Request__rosidl_typesupport_introspection_c__GetRobotGeometry_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_jugglebot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, jugglebot_interfaces, srv, GetRobotGeometry_Request)() {
  if (!GetRobotGeometry_Request__rosidl_typesupport_introspection_c__GetRobotGeometry_Request_message_type_support_handle.typesupport_identifier) {
    GetRobotGeometry_Request__rosidl_typesupport_introspection_c__GetRobotGeometry_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &GetRobotGeometry_Request__rosidl_typesupport_introspection_c__GetRobotGeometry_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "jugglebot_interfaces/srv/detail/get_robot_geometry__rosidl_typesupport_introspection_c.h"
// already included above
// #include "jugglebot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "jugglebot_interfaces/srv/detail/get_robot_geometry__functions.h"
// already included above
// #include "jugglebot_interfaces/srv/detail/get_robot_geometry__struct.h"


// Include directives for member types
// Member `start_pos`
// Member `base_nodes`
// Member `init_plat_nodes`
// Member `init_arm_nodes`
// Member `init_hand_nodes`
// Member `init_leg_lengths`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void GetRobotGeometry_Response__rosidl_typesupport_introspection_c__GetRobotGeometry_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  jugglebot_interfaces__srv__GetRobotGeometry_Response__init(message_memory);
}

void GetRobotGeometry_Response__rosidl_typesupport_introspection_c__GetRobotGeometry_Response_fini_function(void * message_memory)
{
  jugglebot_interfaces__srv__GetRobotGeometry_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember GetRobotGeometry_Response__rosidl_typesupport_introspection_c__GetRobotGeometry_Response_message_member_array[8] = {
  {
    "start_pos",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jugglebot_interfaces__srv__GetRobotGeometry_Response, start_pos),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "base_nodes",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jugglebot_interfaces__srv__GetRobotGeometry_Response, base_nodes),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "init_plat_nodes",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jugglebot_interfaces__srv__GetRobotGeometry_Response, init_plat_nodes),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "init_arm_nodes",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jugglebot_interfaces__srv__GetRobotGeometry_Response, init_arm_nodes),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "init_hand_nodes",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jugglebot_interfaces__srv__GetRobotGeometry_Response, init_hand_nodes),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "init_leg_lengths",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jugglebot_interfaces__srv__GetRobotGeometry_Response, init_leg_lengths),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "leg_stroke",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jugglebot_interfaces__srv__GetRobotGeometry_Response, leg_stroke),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "hand_stroke",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jugglebot_interfaces__srv__GetRobotGeometry_Response, hand_stroke),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers GetRobotGeometry_Response__rosidl_typesupport_introspection_c__GetRobotGeometry_Response_message_members = {
  "jugglebot_interfaces__srv",  // message namespace
  "GetRobotGeometry_Response",  // message name
  8,  // number of fields
  sizeof(jugglebot_interfaces__srv__GetRobotGeometry_Response),
  GetRobotGeometry_Response__rosidl_typesupport_introspection_c__GetRobotGeometry_Response_message_member_array,  // message members
  GetRobotGeometry_Response__rosidl_typesupport_introspection_c__GetRobotGeometry_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  GetRobotGeometry_Response__rosidl_typesupport_introspection_c__GetRobotGeometry_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t GetRobotGeometry_Response__rosidl_typesupport_introspection_c__GetRobotGeometry_Response_message_type_support_handle = {
  0,
  &GetRobotGeometry_Response__rosidl_typesupport_introspection_c__GetRobotGeometry_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_jugglebot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, jugglebot_interfaces, srv, GetRobotGeometry_Response)() {
  if (!GetRobotGeometry_Response__rosidl_typesupport_introspection_c__GetRobotGeometry_Response_message_type_support_handle.typesupport_identifier) {
    GetRobotGeometry_Response__rosidl_typesupport_introspection_c__GetRobotGeometry_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &GetRobotGeometry_Response__rosidl_typesupport_introspection_c__GetRobotGeometry_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "jugglebot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "jugglebot_interfaces/srv/detail/get_robot_geometry__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers jugglebot_interfaces__srv__detail__get_robot_geometry__rosidl_typesupport_introspection_c__GetRobotGeometry_service_members = {
  "jugglebot_interfaces__srv",  // service namespace
  "GetRobotGeometry",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // jugglebot_interfaces__srv__detail__get_robot_geometry__rosidl_typesupport_introspection_c__GetRobotGeometry_Request_message_type_support_handle,
  NULL  // response message
  // jugglebot_interfaces__srv__detail__get_robot_geometry__rosidl_typesupport_introspection_c__GetRobotGeometry_Response_message_type_support_handle
};

static rosidl_service_type_support_t jugglebot_interfaces__srv__detail__get_robot_geometry__rosidl_typesupport_introspection_c__GetRobotGeometry_service_type_support_handle = {
  0,
  &jugglebot_interfaces__srv__detail__get_robot_geometry__rosidl_typesupport_introspection_c__GetRobotGeometry_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, jugglebot_interfaces, srv, GetRobotGeometry_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, jugglebot_interfaces, srv, GetRobotGeometry_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_jugglebot_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, jugglebot_interfaces, srv, GetRobotGeometry)() {
  if (!jugglebot_interfaces__srv__detail__get_robot_geometry__rosidl_typesupport_introspection_c__GetRobotGeometry_service_type_support_handle.typesupport_identifier) {
    jugglebot_interfaces__srv__detail__get_robot_geometry__rosidl_typesupport_introspection_c__GetRobotGeometry_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)jugglebot_interfaces__srv__detail__get_robot_geometry__rosidl_typesupport_introspection_c__GetRobotGeometry_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, jugglebot_interfaces, srv, GetRobotGeometry_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, jugglebot_interfaces, srv, GetRobotGeometry_Response)()->data;
  }

  return &jugglebot_interfaces__srv__detail__get_robot_geometry__rosidl_typesupport_introspection_c__GetRobotGeometry_service_type_support_handle;
}
