// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from jugglebot_interfaces:srv/GetRobotGeometry.idl
// generated code does not contain a copyright notice

#ifndef JUGGLEBOT_INTERFACES__SRV__DETAIL__GET_ROBOT_GEOMETRY__STRUCT_HPP_
#define JUGGLEBOT_INTERFACES__SRV__DETAIL__GET_ROBOT_GEOMETRY__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__jugglebot_interfaces__srv__GetRobotGeometry_Request __attribute__((deprecated))
#else
# define DEPRECATED__jugglebot_interfaces__srv__GetRobotGeometry_Request __declspec(deprecated)
#endif

namespace jugglebot_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetRobotGeometry_Request_
{
  using Type = GetRobotGeometry_Request_<ContainerAllocator>;

  explicit GetRobotGeometry_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit GetRobotGeometry_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    jugglebot_interfaces::srv::GetRobotGeometry_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const jugglebot_interfaces::srv::GetRobotGeometry_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<jugglebot_interfaces::srv::GetRobotGeometry_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<jugglebot_interfaces::srv::GetRobotGeometry_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      jugglebot_interfaces::srv::GetRobotGeometry_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<jugglebot_interfaces::srv::GetRobotGeometry_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      jugglebot_interfaces::srv::GetRobotGeometry_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<jugglebot_interfaces::srv::GetRobotGeometry_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<jugglebot_interfaces::srv::GetRobotGeometry_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<jugglebot_interfaces::srv::GetRobotGeometry_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__jugglebot_interfaces__srv__GetRobotGeometry_Request
    std::shared_ptr<jugglebot_interfaces::srv::GetRobotGeometry_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__jugglebot_interfaces__srv__GetRobotGeometry_Request
    std::shared_ptr<jugglebot_interfaces::srv::GetRobotGeometry_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetRobotGeometry_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetRobotGeometry_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetRobotGeometry_Request_

// alias to use template instance with default allocator
using GetRobotGeometry_Request =
  jugglebot_interfaces::srv::GetRobotGeometry_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace jugglebot_interfaces


#ifndef _WIN32
# define DEPRECATED__jugglebot_interfaces__srv__GetRobotGeometry_Response __attribute__((deprecated))
#else
# define DEPRECATED__jugglebot_interfaces__srv__GetRobotGeometry_Response __declspec(deprecated)
#endif

namespace jugglebot_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetRobotGeometry_Response_
{
  using Type = GetRobotGeometry_Response_<ContainerAllocator>;

  explicit GetRobotGeometry_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->leg_stroke = 0.0f;
    }
  }

  explicit GetRobotGeometry_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->leg_stroke = 0.0f;
    }
  }

  // field types and members
  using _start_pos_type =
    std::vector<double, typename ContainerAllocator::template rebind<double>::other>;
  _start_pos_type start_pos;
  using _base_nodes_type =
    std::vector<double, typename ContainerAllocator::template rebind<double>::other>;
  _base_nodes_type base_nodes;
  using _init_plat_nodes_type =
    std::vector<double, typename ContainerAllocator::template rebind<double>::other>;
  _init_plat_nodes_type init_plat_nodes;
  using _init_arm_nodes_type =
    std::vector<double, typename ContainerAllocator::template rebind<double>::other>;
  _init_arm_nodes_type init_arm_nodes;
  using _init_hand_nodes_type =
    std::vector<double, typename ContainerAllocator::template rebind<double>::other>;
  _init_hand_nodes_type init_hand_nodes;
  using _init_leg_lengths_type =
    std::vector<double, typename ContainerAllocator::template rebind<double>::other>;
  _init_leg_lengths_type init_leg_lengths;
  using _leg_stroke_type =
    float;
  _leg_stroke_type leg_stroke;

  // setters for named parameter idiom
  Type & set__start_pos(
    const std::vector<double, typename ContainerAllocator::template rebind<double>::other> & _arg)
  {
    this->start_pos = _arg;
    return *this;
  }
  Type & set__base_nodes(
    const std::vector<double, typename ContainerAllocator::template rebind<double>::other> & _arg)
  {
    this->base_nodes = _arg;
    return *this;
  }
  Type & set__init_plat_nodes(
    const std::vector<double, typename ContainerAllocator::template rebind<double>::other> & _arg)
  {
    this->init_plat_nodes = _arg;
    return *this;
  }
  Type & set__init_arm_nodes(
    const std::vector<double, typename ContainerAllocator::template rebind<double>::other> & _arg)
  {
    this->init_arm_nodes = _arg;
    return *this;
  }
  Type & set__init_hand_nodes(
    const std::vector<double, typename ContainerAllocator::template rebind<double>::other> & _arg)
  {
    this->init_hand_nodes = _arg;
    return *this;
  }
  Type & set__init_leg_lengths(
    const std::vector<double, typename ContainerAllocator::template rebind<double>::other> & _arg)
  {
    this->init_leg_lengths = _arg;
    return *this;
  }
  Type & set__leg_stroke(
    const float & _arg)
  {
    this->leg_stroke = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    jugglebot_interfaces::srv::GetRobotGeometry_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const jugglebot_interfaces::srv::GetRobotGeometry_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<jugglebot_interfaces::srv::GetRobotGeometry_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<jugglebot_interfaces::srv::GetRobotGeometry_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      jugglebot_interfaces::srv::GetRobotGeometry_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<jugglebot_interfaces::srv::GetRobotGeometry_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      jugglebot_interfaces::srv::GetRobotGeometry_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<jugglebot_interfaces::srv::GetRobotGeometry_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<jugglebot_interfaces::srv::GetRobotGeometry_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<jugglebot_interfaces::srv::GetRobotGeometry_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__jugglebot_interfaces__srv__GetRobotGeometry_Response
    std::shared_ptr<jugglebot_interfaces::srv::GetRobotGeometry_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__jugglebot_interfaces__srv__GetRobotGeometry_Response
    std::shared_ptr<jugglebot_interfaces::srv::GetRobotGeometry_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetRobotGeometry_Response_ & other) const
  {
    if (this->start_pos != other.start_pos) {
      return false;
    }
    if (this->base_nodes != other.base_nodes) {
      return false;
    }
    if (this->init_plat_nodes != other.init_plat_nodes) {
      return false;
    }
    if (this->init_arm_nodes != other.init_arm_nodes) {
      return false;
    }
    if (this->init_hand_nodes != other.init_hand_nodes) {
      return false;
    }
    if (this->init_leg_lengths != other.init_leg_lengths) {
      return false;
    }
    if (this->leg_stroke != other.leg_stroke) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetRobotGeometry_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetRobotGeometry_Response_

// alias to use template instance with default allocator
using GetRobotGeometry_Response =
  jugglebot_interfaces::srv::GetRobotGeometry_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace jugglebot_interfaces

namespace jugglebot_interfaces
{

namespace srv
{

struct GetRobotGeometry
{
  using Request = jugglebot_interfaces::srv::GetRobotGeometry_Request;
  using Response = jugglebot_interfaces::srv::GetRobotGeometry_Response;
};

}  // namespace srv

}  // namespace jugglebot_interfaces

#endif  // JUGGLEBOT_INTERFACES__SRV__DETAIL__GET_ROBOT_GEOMETRY__STRUCT_HPP_
