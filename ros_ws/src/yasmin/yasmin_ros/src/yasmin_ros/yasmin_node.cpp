// Copyright (C) 2024  Miguel Ángel González Santamarta

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <iomanip>
#include <random>
#include <sstream>

#include "yasmin_ros/yasmin_node.hpp"

using namespace yasmin_ros;

std::string generateUUID() {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, 15);
  auto rand_hex_digit = [&gen, &dis]() {
    constexpr char hex_digits[] = "0123456789abcdef";
    return hex_digits[dis(gen)];
  };

  std::stringstream ss;
  for (int i = 0; i < 32; ++i) {
    if (i == 8 || i == 12 || i == 16 || i == 20)
      ss << "_";
    ss << rand_hex_digit();
  }
  return ss.str();
}

YasminNode::YasminNode() : rclcpp::Node("yasmin_" + generateUUID() + "_node") {

  this->executor.add_node(this->get_node_base_interface());
  this->spin_thread = std::make_unique<std::thread>(
      &rclcpp::executors::MultiThreadedExecutor::spin, &this->executor);
  this->spin_thread->detach();
}
