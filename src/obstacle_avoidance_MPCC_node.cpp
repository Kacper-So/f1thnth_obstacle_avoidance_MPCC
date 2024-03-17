// Copyright 2024 g03
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "obstacle_avoidance_MPCC/obstacle_avoidance_MPCC_node.hpp"

namespace obstacle_avoidance_MPCC
{

ObstacleAvoidanceMpccNode::ObstacleAvoidanceMpccNode(const rclcpp::NodeOptions & options)
:  Node("obstacle_avoidance_MPCC", options)
{
  obstacle_avoidance_MPCC_ = std::make_unique<obstacle_avoidance_MPCC::ObstacleAvoidanceMpcc>();
  param_name_ = this->declare_parameter("param_name", 456);
  obstacle_avoidance_MPCC_->foo(param_name_);
}

}  // namespace obstacle_avoidance_MPCC

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(obstacle_avoidance_MPCC::ObstacleAvoidanceMpccNode)
