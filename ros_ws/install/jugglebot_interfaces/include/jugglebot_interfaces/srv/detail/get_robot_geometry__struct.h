// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from jugglebot_interfaces:srv/GetRobotGeometry.idl
// generated code does not contain a copyright notice

#ifndef JUGGLEBOT_INTERFACES__SRV__DETAIL__GET_ROBOT_GEOMETRY__STRUCT_H_
#define JUGGLEBOT_INTERFACES__SRV__DETAIL__GET_ROBOT_GEOMETRY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in srv/GetRobotGeometry in the package jugglebot_interfaces.
typedef struct jugglebot_interfaces__srv__GetRobotGeometry_Request
{
  uint8_t structure_needs_at_least_one_member;
} jugglebot_interfaces__srv__GetRobotGeometry_Request;

// Struct for a sequence of jugglebot_interfaces__srv__GetRobotGeometry_Request.
typedef struct jugglebot_interfaces__srv__GetRobotGeometry_Request__Sequence
{
  jugglebot_interfaces__srv__GetRobotGeometry_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jugglebot_interfaces__srv__GetRobotGeometry_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'start_pos'
// Member 'base_nodes'
// Member 'init_plat_nodes'
// Member 'init_arm_nodes'
// Member 'init_hand_nodes'
// Member 'init_leg_lengths'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in srv/GetRobotGeometry in the package jugglebot_interfaces.
typedef struct jugglebot_interfaces__srv__GetRobotGeometry_Response
{
  rosidl_runtime_c__double__Sequence start_pos;
  rosidl_runtime_c__double__Sequence base_nodes;
  rosidl_runtime_c__double__Sequence init_plat_nodes;
  rosidl_runtime_c__double__Sequence init_arm_nodes;
  rosidl_runtime_c__double__Sequence init_hand_nodes;
  rosidl_runtime_c__double__Sequence init_leg_lengths;
  float leg_stroke;
  float hand_stroke;
} jugglebot_interfaces__srv__GetRobotGeometry_Response;

// Struct for a sequence of jugglebot_interfaces__srv__GetRobotGeometry_Response.
typedef struct jugglebot_interfaces__srv__GetRobotGeometry_Response__Sequence
{
  jugglebot_interfaces__srv__GetRobotGeometry_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jugglebot_interfaces__srv__GetRobotGeometry_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // JUGGLEBOT_INTERFACES__SRV__DETAIL__GET_ROBOT_GEOMETRY__STRUCT_H_
