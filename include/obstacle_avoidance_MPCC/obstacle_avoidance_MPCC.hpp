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

#ifndef OBSTACLE_AVOIDANCE_MPCC__OBSTACLE_AVOIDANCE_MPCC_HPP_
#define OBSTACLE_AVOIDANCE_MPCC__OBSTACLE_AVOIDANCE_MPCC_HPP_

#include <cstdint>

#include "obstacle_avoidance_MPCC/visibility_control.hpp"


namespace obstacle_avoidance_MPCC
{

class OBSTACLE_AVOIDANCE_MPCC_PUBLIC ObstacleAvoidanceMpcc
{
public:
  ObstacleAvoidanceMpcc();
  int64_t foo(int64_t bar) const;
};

}  // namespace obstacle_avoidance_MPCC

#endif  // OBSTACLE_AVOIDANCE_MPCC__OBSTACLE_AVOIDANCE_MPCC_HPP_
