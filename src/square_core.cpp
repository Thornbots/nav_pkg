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

#include "thornbots_nav/square_core.hpp"

#include <array>
#include <cmath>
#include <stdexcept>

namespace thornbots_nav
{

namespace
{
// Corner offsets in units of the side, in visiting order.
constexpr std::array<std::array<double, 2>, 4> kCorners{{{0, 0}, {1, 0}, {1, 1}, {0, 1}}};
}  // namespace

SquareCore::SquareCore(double side_m, double tolerance_m)
: side_(side_m), tolerance_(tolerance_m)
{
  if (!std::isfinite(side_m) || !(side_m > 0.0) || !std::isfinite(tolerance_m) ||
    !(tolerance_m > 0.0) || !(tolerance_m < side_m / 2))
  {
    throw std::invalid_argument("need square_side_m > 0 and 0 < square_tolerance_m < side / 2");
  }
}

void SquareCore::on_pose(double x, double y)
{
  x_ = x;
  y_ = y;
  if (!x0_) {
    x0_ = x;
    y0_ = y;
  }
  const auto g = goal();
  if (std::hypot(x_ - g->x, y_ - g->y) <= tolerance_) {
    target_ = (target_ + 1) % kCorners.size();
  }
}

std::optional<Goal> SquareCore::goal() const
{
  if (!x0_) {
    return std::nullopt;
  }
  const auto & c = kCorners[target_];
  return Goal{*x0_ + side_ * c[0], *y0_ + side_ * c[1], target_};
}

}  // namespace thornbots_nav
