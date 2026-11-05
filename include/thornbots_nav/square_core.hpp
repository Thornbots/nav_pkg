// Copyright 2026 Thornbots
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// ROS-free off-field test route: a square anchored at the first pose.

#pragma once

#include <cstddef>
#include <optional>

#include "thornbots_nav/route_core.hpp"

namespace thornbots_nav
{

// Loops start -> (+side, 0) -> (+side, +side) -> (0, +side) -> start, in field
// +x/+y for either team (no mirroring), ignoring HP and game stage.
class SquareCore
{
public:
  // Throws std::invalid_argument unless side_m and tolerance_m are finite and
  // > 0 and tolerance_m < side_m / 2.
  SquareCore(double side_m, double tolerance_m);

  void on_pose(double x, double y);

  // Empty until the first pose, which becomes corner 0.
  std::optional<Goal> goal() const;

private:
  double side_;
  double tolerance_;
  std::optional<double> x0_, y0_;
  double x_{0.0}, y_{0.0};
  std::size_t target_{1};
};

}  // namespace thornbots_nav
