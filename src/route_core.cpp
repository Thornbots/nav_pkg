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

// Ported from the MCB's SimpleAutoDriveCommand, minus its stuck timer and spin.

#include "thornbots_nav/route_core.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace thornbots_nav
{

namespace
{
bool all_finite(const std::vector<double> & values)
{
  for (const double v : values) {
    if (!std::isfinite(v)) {
      return false;
    }
  }
  return true;
}
}  // namespace

void validate(const RouteParams & p)
{
  if (p.x.size() < 2 || p.x.size() != p.y.size() || p.x.size() != p.tolerance_m.size()) {
    throw std::invalid_argument(
            "waypoints_x, waypoints_y and tolerances_m must have equal length >= 2");
  }
  if (!all_finite(p.x) || !all_finite(p.y) || !all_finite(p.tolerance_m)) {
    throw std::invalid_argument("waypoints and tolerances must be finite");
  }
  for (const double t : p.tolerance_m) {
    if (!(t > 0.0)) {
      throw std::invalid_argument("tolerances_m must be > 0");
    }
  }
  if (!std::isfinite(p.max_hp) || !(p.max_hp > 0.0)) {
    throw std::invalid_argument("max_hp must be finite and > 0");
  }
  if (!std::isfinite(p.retreat_hp_ratio) || !std::isfinite(p.advance_hp_ratio) ||
    !(p.retreat_hp_ratio > 0.0) || !(p.retreat_hp_ratio < p.advance_hp_ratio) ||
    !(p.advance_hp_ratio <= 1.0))
  {
    throw std::invalid_argument("need 0 < retreat_hp_ratio < advance_hp_ratio <= 1");
  }
}

RouteCore::RouteCore(RouteParams params)
: p_(std::move(params))
{
  validate(p_);
}

void RouteCore::on_ref_sys(bool is_blue, double hp, int game_stage)
{
  have_ref_ = true;
  is_blue_ = is_blue;
  hp_ = hp;
  stage_ = game_stage;
  step();
}

void RouteCore::on_pose(double x, double y)
{
  have_pose_ = true;
  x_ = x;
  y_ = y;
  step();
}

std::optional<Goal> RouteCore::goal() const
{
  if (!have_ref_ || !have_pose_) {
    return std::nullopt;
  }
  const double sign = is_blue_ ? -1.0 : 1.0;
  return Goal{sign * p_.x[target_], p_.y[target_], target_};
}

void RouteCore::step()
{
  if (!have_ref_ || !have_pose_) {
    return;
  }
  const bool in_game = stage_ == p_.in_game_stage;
  const int last = static_cast<int>(p_.x.size()) - 1;

  int wanted = direction_;
  if (hp_ > p_.advance_hp_ratio * p_.max_hp) {
    wanted = 1;
  } else if (hp_ <= p_.retreat_hp_ratio * p_.max_hp) {
    wanted = -1;
  }
  // A flip re-traces the path: the next waypoint in the new direction is the
  // one just left. Heading out again waits for the game, like advancing does.
  if (wanted != direction_ && (wanted < 0 || in_game)) {
    direction_ = wanted;
    const int flipped = static_cast<int>(target_) + direction_;
    target_ = static_cast<std::size_t>(flipped < 0 ? 0 : (flipped > last ? last : flipped));
  }

  const int next = static_cast<int>(target_) + direction_;
  if (next < 0 || next > last || (direction_ > 0 && !in_game)) {
    return;
  }
  const double sign = is_blue_ ? -1.0 : 1.0;
  const double dist = std::hypot(x_ - sign * p_.x[target_], y_ - p_.y[target_]);
  if (dist <= p_.tolerance_m[target_]) {
    target_ = static_cast<std::size_t>(next);
  }
}

}  // namespace thornbots_nav
