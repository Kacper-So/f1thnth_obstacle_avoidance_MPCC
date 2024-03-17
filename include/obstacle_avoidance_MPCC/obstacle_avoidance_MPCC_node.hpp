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

#ifndef OBSTACLE_AVOIDANCE_MPCC__OBSTACLE_AVOIDANCE_MPCC_NODE_HPP_
#define OBSTACLE_AVOIDANCE_MPCC__OBSTACLE_AVOIDANCE_MPCC_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "obstacle_avoidance_MPCC/obstacle_avoidance_MPCC.hpp"

namespace obstacle_avoidance_MPCC
{
using ObstacleAvoidanceMpccPtr = std::unique_ptr<obstacle_avoidance_MPCC::ObstacleAvoidanceMpcc>;

class OBSTACLE_AVOIDANCE_MPCC_PUBLIC ObstacleAvoidanceMpccNode : public rclcpp::Node
{
public:
  explicit ObstacleAvoidanceMpccNode(const rclcpp::NodeOptions & options);

private:
  ObstacleAvoidanceMpccPtr obstacle_avoidance_MPCC_{nullptr};
  int64_t param_name_{123};
};
}  // namespace obstacle_avoidance_MPCC

#endif  // OBSTACLE_AVOIDANCE_MPCC__OBSTACLE_AVOIDANCE_MPCC_NODE_HPP_
