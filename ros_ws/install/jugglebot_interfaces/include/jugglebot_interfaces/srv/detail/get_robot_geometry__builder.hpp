// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from jugglebot_interfaces:srv/GetRobotGeometry.idl
// generated code does not contain a copyright notice

#ifndef JUGGLEBOT_INTERFACES__SRV__DETAIL__GET_ROBOT_GEOMETRY__BUILDER_HPP_
#define JUGGLEBOT_INTERFACES__SRV__DETAIL__GET_ROBOT_GEOMETRY__BUILDER_HPP_

#include "jugglebot_interfaces/srv/detail/get_robot_geometry__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace jugglebot_interfaces
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::jugglebot_interfaces::srv::GetRobotGeometry_Request>()
{
  return ::jugglebot_interfaces::srv::GetRobotGeometry_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace jugglebot_interfaces


namespace jugglebot_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetRobotGeometry_Response_hand_stroke
{
public:
  explicit Init_GetRobotGeometry_Response_hand_stroke(::jugglebot_interfaces::srv::GetRobotGeometry_Response & msg)
  : msg_(msg)
  {}
  ::jugglebot_interfaces::srv::GetRobotGeometry_Response hand_stroke(::jugglebot_interfaces::srv::GetRobotGeometry_Response::_hand_stroke_type arg)
  {
    msg_.hand_stroke = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jugglebot_interfaces::srv::GetRobotGeometry_Response msg_;
};

class Init_GetRobotGeometry_Response_leg_stroke
{
public:
  explicit Init_GetRobotGeometry_Response_leg_stroke(::jugglebot_interfaces::srv::GetRobotGeometry_Response & msg)
  : msg_(msg)
  {}
  Init_GetRobotGeometry_Response_hand_stroke leg_stroke(::jugglebot_interfaces::srv::GetRobotGeometry_Response::_leg_stroke_type arg)
  {
    msg_.leg_stroke = std::move(arg);
    return Init_GetRobotGeometry_Response_hand_stroke(msg_);
  }

private:
  ::jugglebot_interfaces::srv::GetRobotGeometry_Response msg_;
};

class Init_GetRobotGeometry_Response_init_leg_lengths
{
public:
  explicit Init_GetRobotGeometry_Response_init_leg_lengths(::jugglebot_interfaces::srv::GetRobotGeometry_Response & msg)
  : msg_(msg)
  {}
  Init_GetRobotGeometry_Response_leg_stroke init_leg_lengths(::jugglebot_interfaces::srv::GetRobotGeometry_Response::_init_leg_lengths_type arg)
  {
    msg_.init_leg_lengths = std::move(arg);
    return Init_GetRobotGeometry_Response_leg_stroke(msg_);
  }

private:
  ::jugglebot_interfaces::srv::GetRobotGeometry_Response msg_;
};

class Init_GetRobotGeometry_Response_init_hand_nodes
{
public:
  explicit Init_GetRobotGeometry_Response_init_hand_nodes(::jugglebot_interfaces::srv::GetRobotGeometry_Response & msg)
  : msg_(msg)
  {}
  Init_GetRobotGeometry_Response_init_leg_lengths init_hand_nodes(::jugglebot_interfaces::srv::GetRobotGeometry_Response::_init_hand_nodes_type arg)
  {
    msg_.init_hand_nodes = std::move(arg);
    return Init_GetRobotGeometry_Response_init_leg_lengths(msg_);
  }

private:
  ::jugglebot_interfaces::srv::GetRobotGeometry_Response msg_;
};

class Init_GetRobotGeometry_Response_init_arm_nodes
{
public:
  explicit Init_GetRobotGeometry_Response_init_arm_nodes(::jugglebot_interfaces::srv::GetRobotGeometry_Response & msg)
  : msg_(msg)
  {}
  Init_GetRobotGeometry_Response_init_hand_nodes init_arm_nodes(::jugglebot_interfaces::srv::GetRobotGeometry_Response::_init_arm_nodes_type arg)
  {
    msg_.init_arm_nodes = std::move(arg);
    return Init_GetRobotGeometry_Response_init_hand_nodes(msg_);
  }

private:
  ::jugglebot_interfaces::srv::GetRobotGeometry_Response msg_;
};

class Init_GetRobotGeometry_Response_init_plat_nodes
{
public:
  explicit Init_GetRobotGeometry_Response_init_plat_nodes(::jugglebot_interfaces::srv::GetRobotGeometry_Response & msg)
  : msg_(msg)
  {}
  Init_GetRobotGeometry_Response_init_arm_nodes init_plat_nodes(::jugglebot_interfaces::srv::GetRobotGeometry_Response::_init_plat_nodes_type arg)
  {
    msg_.init_plat_nodes = std::move(arg);
    return Init_GetRobotGeometry_Response_init_arm_nodes(msg_);
  }

private:
  ::jugglebot_interfaces::srv::GetRobotGeometry_Response msg_;
};

class Init_GetRobotGeometry_Response_base_nodes
{
public:
  explicit Init_GetRobotGeometry_Response_base_nodes(::jugglebot_interfaces::srv::GetRobotGeometry_Response & msg)
  : msg_(msg)
  {}
  Init_GetRobotGeometry_Response_init_plat_nodes base_nodes(::jugglebot_interfaces::srv::GetRobotGeometry_Response::_base_nodes_type arg)
  {
    msg_.base_nodes = std::move(arg);
    return Init_GetRobotGeometry_Response_init_plat_nodes(msg_);
  }

private:
  ::jugglebot_interfaces::srv::GetRobotGeometry_Response msg_;
};

class Init_GetRobotGeometry_Response_start_pos
{
public:
  Init_GetRobotGeometry_Response_start_pos()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetRobotGeometry_Response_base_nodes start_pos(::jugglebot_interfaces::srv::GetRobotGeometry_Response::_start_pos_type arg)
  {
    msg_.start_pos = std::move(arg);
    return Init_GetRobotGeometry_Response_base_nodes(msg_);
  }

private:
  ::jugglebot_interfaces::srv::GetRobotGeometry_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::jugglebot_interfaces::srv::GetRobotGeometry_Response>()
{
  return jugglebot_interfaces::srv::builder::Init_GetRobotGeometry_Response_start_pos();
}

}  // namespace jugglebot_interfaces

#endif  // JUGGLEBOT_INTERFACES__SRV__DETAIL__GET_ROBOT_GEOMETRY__BUILDER_HPP_
