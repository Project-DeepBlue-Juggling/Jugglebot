// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from jugglebot_interfaces:srv/GetRobotGeometry.idl
// generated code does not contain a copyright notice

#ifndef JUGGLEBOT_INTERFACES__SRV__DETAIL__GET_ROBOT_GEOMETRY__TRAITS_HPP_
#define JUGGLEBOT_INTERFACES__SRV__DETAIL__GET_ROBOT_GEOMETRY__TRAITS_HPP_

#include "jugglebot_interfaces/srv/detail/get_robot_geometry__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<jugglebot_interfaces::srv::GetRobotGeometry_Request>()
{
  return "jugglebot_interfaces::srv::GetRobotGeometry_Request";
}

template<>
inline const char * name<jugglebot_interfaces::srv::GetRobotGeometry_Request>()
{
  return "jugglebot_interfaces/srv/GetRobotGeometry_Request";
}

template<>
struct has_fixed_size<jugglebot_interfaces::srv::GetRobotGeometry_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<jugglebot_interfaces::srv::GetRobotGeometry_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<jugglebot_interfaces::srv::GetRobotGeometry_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<jugglebot_interfaces::srv::GetRobotGeometry_Response>()
{
  return "jugglebot_interfaces::srv::GetRobotGeometry_Response";
}

template<>
inline const char * name<jugglebot_interfaces::srv::GetRobotGeometry_Response>()
{
  return "jugglebot_interfaces/srv/GetRobotGeometry_Response";
}

template<>
struct has_fixed_size<jugglebot_interfaces::srv::GetRobotGeometry_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<jugglebot_interfaces::srv::GetRobotGeometry_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<jugglebot_interfaces::srv::GetRobotGeometry_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<jugglebot_interfaces::srv::GetRobotGeometry>()
{
  return "jugglebot_interfaces::srv::GetRobotGeometry";
}

template<>
inline const char * name<jugglebot_interfaces::srv::GetRobotGeometry>()
{
  return "jugglebot_interfaces/srv/GetRobotGeometry";
}

template<>
struct has_fixed_size<jugglebot_interfaces::srv::GetRobotGeometry>
  : std::integral_constant<
    bool,
    has_fixed_size<jugglebot_interfaces::srv::GetRobotGeometry_Request>::value &&
    has_fixed_size<jugglebot_interfaces::srv::GetRobotGeometry_Response>::value
  >
{
};

template<>
struct has_bounded_size<jugglebot_interfaces::srv::GetRobotGeometry>
  : std::integral_constant<
    bool,
    has_bounded_size<jugglebot_interfaces::srv::GetRobotGeometry_Request>::value &&
    has_bounded_size<jugglebot_interfaces::srv::GetRobotGeometry_Response>::value
  >
{
};

template<>
struct is_service<jugglebot_interfaces::srv::GetRobotGeometry>
  : std::true_type
{
};

template<>
struct is_service_request<jugglebot_interfaces::srv::GetRobotGeometry_Request>
  : std::true_type
{
};

template<>
struct is_service_response<jugglebot_interfaces::srv::GetRobotGeometry_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // JUGGLEBOT_INTERFACES__SRV__DETAIL__GET_ROBOT_GEOMETRY__TRAITS_HPP_
