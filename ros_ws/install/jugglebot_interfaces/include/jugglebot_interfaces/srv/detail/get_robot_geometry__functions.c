// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from jugglebot_interfaces:srv/GetRobotGeometry.idl
// generated code does not contain a copyright notice
#include "jugglebot_interfaces/srv/detail/get_robot_geometry__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
jugglebot_interfaces__srv__GetRobotGeometry_Request__init(jugglebot_interfaces__srv__GetRobotGeometry_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
jugglebot_interfaces__srv__GetRobotGeometry_Request__fini(jugglebot_interfaces__srv__GetRobotGeometry_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
jugglebot_interfaces__srv__GetRobotGeometry_Request__are_equal(const jugglebot_interfaces__srv__GetRobotGeometry_Request * lhs, const jugglebot_interfaces__srv__GetRobotGeometry_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
jugglebot_interfaces__srv__GetRobotGeometry_Request__copy(
  const jugglebot_interfaces__srv__GetRobotGeometry_Request * input,
  jugglebot_interfaces__srv__GetRobotGeometry_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

jugglebot_interfaces__srv__GetRobotGeometry_Request *
jugglebot_interfaces__srv__GetRobotGeometry_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jugglebot_interfaces__srv__GetRobotGeometry_Request * msg = (jugglebot_interfaces__srv__GetRobotGeometry_Request *)allocator.allocate(sizeof(jugglebot_interfaces__srv__GetRobotGeometry_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(jugglebot_interfaces__srv__GetRobotGeometry_Request));
  bool success = jugglebot_interfaces__srv__GetRobotGeometry_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
jugglebot_interfaces__srv__GetRobotGeometry_Request__destroy(jugglebot_interfaces__srv__GetRobotGeometry_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    jugglebot_interfaces__srv__GetRobotGeometry_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
jugglebot_interfaces__srv__GetRobotGeometry_Request__Sequence__init(jugglebot_interfaces__srv__GetRobotGeometry_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jugglebot_interfaces__srv__GetRobotGeometry_Request * data = NULL;

  if (size) {
    data = (jugglebot_interfaces__srv__GetRobotGeometry_Request *)allocator.zero_allocate(size, sizeof(jugglebot_interfaces__srv__GetRobotGeometry_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = jugglebot_interfaces__srv__GetRobotGeometry_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        jugglebot_interfaces__srv__GetRobotGeometry_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
jugglebot_interfaces__srv__GetRobotGeometry_Request__Sequence__fini(jugglebot_interfaces__srv__GetRobotGeometry_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      jugglebot_interfaces__srv__GetRobotGeometry_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

jugglebot_interfaces__srv__GetRobotGeometry_Request__Sequence *
jugglebot_interfaces__srv__GetRobotGeometry_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jugglebot_interfaces__srv__GetRobotGeometry_Request__Sequence * array = (jugglebot_interfaces__srv__GetRobotGeometry_Request__Sequence *)allocator.allocate(sizeof(jugglebot_interfaces__srv__GetRobotGeometry_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = jugglebot_interfaces__srv__GetRobotGeometry_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
jugglebot_interfaces__srv__GetRobotGeometry_Request__Sequence__destroy(jugglebot_interfaces__srv__GetRobotGeometry_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    jugglebot_interfaces__srv__GetRobotGeometry_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
jugglebot_interfaces__srv__GetRobotGeometry_Request__Sequence__are_equal(const jugglebot_interfaces__srv__GetRobotGeometry_Request__Sequence * lhs, const jugglebot_interfaces__srv__GetRobotGeometry_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!jugglebot_interfaces__srv__GetRobotGeometry_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
jugglebot_interfaces__srv__GetRobotGeometry_Request__Sequence__copy(
  const jugglebot_interfaces__srv__GetRobotGeometry_Request__Sequence * input,
  jugglebot_interfaces__srv__GetRobotGeometry_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(jugglebot_interfaces__srv__GetRobotGeometry_Request);
    jugglebot_interfaces__srv__GetRobotGeometry_Request * data =
      (jugglebot_interfaces__srv__GetRobotGeometry_Request *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!jugglebot_interfaces__srv__GetRobotGeometry_Request__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          jugglebot_interfaces__srv__GetRobotGeometry_Request__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!jugglebot_interfaces__srv__GetRobotGeometry_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `start_pos`
// Member `base_nodes`
// Member `init_plat_nodes`
// Member `init_arm_nodes`
// Member `init_hand_nodes`
// Member `init_leg_lengths`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
jugglebot_interfaces__srv__GetRobotGeometry_Response__init(jugglebot_interfaces__srv__GetRobotGeometry_Response * msg)
{
  if (!msg) {
    return false;
  }
  // start_pos
  if (!rosidl_runtime_c__double__Sequence__init(&msg->start_pos, 0)) {
    jugglebot_interfaces__srv__GetRobotGeometry_Response__fini(msg);
    return false;
  }
  // base_nodes
  if (!rosidl_runtime_c__double__Sequence__init(&msg->base_nodes, 0)) {
    jugglebot_interfaces__srv__GetRobotGeometry_Response__fini(msg);
    return false;
  }
  // init_plat_nodes
  if (!rosidl_runtime_c__double__Sequence__init(&msg->init_plat_nodes, 0)) {
    jugglebot_interfaces__srv__GetRobotGeometry_Response__fini(msg);
    return false;
  }
  // init_arm_nodes
  if (!rosidl_runtime_c__double__Sequence__init(&msg->init_arm_nodes, 0)) {
    jugglebot_interfaces__srv__GetRobotGeometry_Response__fini(msg);
    return false;
  }
  // init_hand_nodes
  if (!rosidl_runtime_c__double__Sequence__init(&msg->init_hand_nodes, 0)) {
    jugglebot_interfaces__srv__GetRobotGeometry_Response__fini(msg);
    return false;
  }
  // init_leg_lengths
  if (!rosidl_runtime_c__double__Sequence__init(&msg->init_leg_lengths, 0)) {
    jugglebot_interfaces__srv__GetRobotGeometry_Response__fini(msg);
    return false;
  }
  // leg_stroke
  // hand_stroke
  return true;
}

void
jugglebot_interfaces__srv__GetRobotGeometry_Response__fini(jugglebot_interfaces__srv__GetRobotGeometry_Response * msg)
{
  if (!msg) {
    return;
  }
  // start_pos
  rosidl_runtime_c__double__Sequence__fini(&msg->start_pos);
  // base_nodes
  rosidl_runtime_c__double__Sequence__fini(&msg->base_nodes);
  // init_plat_nodes
  rosidl_runtime_c__double__Sequence__fini(&msg->init_plat_nodes);
  // init_arm_nodes
  rosidl_runtime_c__double__Sequence__fini(&msg->init_arm_nodes);
  // init_hand_nodes
  rosidl_runtime_c__double__Sequence__fini(&msg->init_hand_nodes);
  // init_leg_lengths
  rosidl_runtime_c__double__Sequence__fini(&msg->init_leg_lengths);
  // leg_stroke
  // hand_stroke
}

bool
jugglebot_interfaces__srv__GetRobotGeometry_Response__are_equal(const jugglebot_interfaces__srv__GetRobotGeometry_Response * lhs, const jugglebot_interfaces__srv__GetRobotGeometry_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // start_pos
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->start_pos), &(rhs->start_pos)))
  {
    return false;
  }
  // base_nodes
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->base_nodes), &(rhs->base_nodes)))
  {
    return false;
  }
  // init_plat_nodes
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->init_plat_nodes), &(rhs->init_plat_nodes)))
  {
    return false;
  }
  // init_arm_nodes
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->init_arm_nodes), &(rhs->init_arm_nodes)))
  {
    return false;
  }
  // init_hand_nodes
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->init_hand_nodes), &(rhs->init_hand_nodes)))
  {
    return false;
  }
  // init_leg_lengths
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->init_leg_lengths), &(rhs->init_leg_lengths)))
  {
    return false;
  }
  // leg_stroke
  if (lhs->leg_stroke != rhs->leg_stroke) {
    return false;
  }
  // hand_stroke
  if (lhs->hand_stroke != rhs->hand_stroke) {
    return false;
  }
  return true;
}

bool
jugglebot_interfaces__srv__GetRobotGeometry_Response__copy(
  const jugglebot_interfaces__srv__GetRobotGeometry_Response * input,
  jugglebot_interfaces__srv__GetRobotGeometry_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // start_pos
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->start_pos), &(output->start_pos)))
  {
    return false;
  }
  // base_nodes
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->base_nodes), &(output->base_nodes)))
  {
    return false;
  }
  // init_plat_nodes
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->init_plat_nodes), &(output->init_plat_nodes)))
  {
    return false;
  }
  // init_arm_nodes
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->init_arm_nodes), &(output->init_arm_nodes)))
  {
    return false;
  }
  // init_hand_nodes
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->init_hand_nodes), &(output->init_hand_nodes)))
  {
    return false;
  }
  // init_leg_lengths
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->init_leg_lengths), &(output->init_leg_lengths)))
  {
    return false;
  }
  // leg_stroke
  output->leg_stroke = input->leg_stroke;
  // hand_stroke
  output->hand_stroke = input->hand_stroke;
  return true;
}

jugglebot_interfaces__srv__GetRobotGeometry_Response *
jugglebot_interfaces__srv__GetRobotGeometry_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jugglebot_interfaces__srv__GetRobotGeometry_Response * msg = (jugglebot_interfaces__srv__GetRobotGeometry_Response *)allocator.allocate(sizeof(jugglebot_interfaces__srv__GetRobotGeometry_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(jugglebot_interfaces__srv__GetRobotGeometry_Response));
  bool success = jugglebot_interfaces__srv__GetRobotGeometry_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
jugglebot_interfaces__srv__GetRobotGeometry_Response__destroy(jugglebot_interfaces__srv__GetRobotGeometry_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    jugglebot_interfaces__srv__GetRobotGeometry_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
jugglebot_interfaces__srv__GetRobotGeometry_Response__Sequence__init(jugglebot_interfaces__srv__GetRobotGeometry_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jugglebot_interfaces__srv__GetRobotGeometry_Response * data = NULL;

  if (size) {
    data = (jugglebot_interfaces__srv__GetRobotGeometry_Response *)allocator.zero_allocate(size, sizeof(jugglebot_interfaces__srv__GetRobotGeometry_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = jugglebot_interfaces__srv__GetRobotGeometry_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        jugglebot_interfaces__srv__GetRobotGeometry_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
jugglebot_interfaces__srv__GetRobotGeometry_Response__Sequence__fini(jugglebot_interfaces__srv__GetRobotGeometry_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      jugglebot_interfaces__srv__GetRobotGeometry_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

jugglebot_interfaces__srv__GetRobotGeometry_Response__Sequence *
jugglebot_interfaces__srv__GetRobotGeometry_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jugglebot_interfaces__srv__GetRobotGeometry_Response__Sequence * array = (jugglebot_interfaces__srv__GetRobotGeometry_Response__Sequence *)allocator.allocate(sizeof(jugglebot_interfaces__srv__GetRobotGeometry_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = jugglebot_interfaces__srv__GetRobotGeometry_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
jugglebot_interfaces__srv__GetRobotGeometry_Response__Sequence__destroy(jugglebot_interfaces__srv__GetRobotGeometry_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    jugglebot_interfaces__srv__GetRobotGeometry_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
jugglebot_interfaces__srv__GetRobotGeometry_Response__Sequence__are_equal(const jugglebot_interfaces__srv__GetRobotGeometry_Response__Sequence * lhs, const jugglebot_interfaces__srv__GetRobotGeometry_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!jugglebot_interfaces__srv__GetRobotGeometry_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
jugglebot_interfaces__srv__GetRobotGeometry_Response__Sequence__copy(
  const jugglebot_interfaces__srv__GetRobotGeometry_Response__Sequence * input,
  jugglebot_interfaces__srv__GetRobotGeometry_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(jugglebot_interfaces__srv__GetRobotGeometry_Response);
    jugglebot_interfaces__srv__GetRobotGeometry_Response * data =
      (jugglebot_interfaces__srv__GetRobotGeometry_Response *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!jugglebot_interfaces__srv__GetRobotGeometry_Response__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          jugglebot_interfaces__srv__GetRobotGeometry_Response__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!jugglebot_interfaces__srv__GetRobotGeometry_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
