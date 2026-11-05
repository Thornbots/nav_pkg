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

// ROS-free waypoint route logic; see README.md for the behavior table.

#pragma once

#include <cstddef>
#include <optional>
#include <vector>

namespace thornbots_nav
{

// Red-team field coordinates; blue mirrors x. Defaults live in config/route.yaml.
struct RouteParams
{
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> tolerance_m;  // per-waypoint arrival radius
  double max_hp{0.0};
  double retreat_hp_ratio{0.0};     // go home at hp <= ratio * max_hp
  double advance_hp_ratio{0.0};     // head out again at hp > ratio * max_hp
  int in_game_stage{0};
};

// Throws std::invalid_argument unless the arrays match (>= 2 points), every
// value is finite, tolerances and max_hp are > 0 and 0 < retreat < advance <= 1.
void validate(const RouteParams & params);

struct Goal
{
  double x;
  double y;
  std::size_t index;
};

class RouteCore
{
public:
  // Validates; throws std::invalid_argument.
  explicit RouteCore(RouteParams params);

  void on_ref_sys(bool is_blue, double hp, int game_stage);
  void on_pose(double x, double y);

  // Empty until both a REF_SYS and a pose have arrived (the team is unknown before).
  std::optional<Goal> goal() const;
  int direction() const {return direction_;}

private:
  void step();

  RouteParams p_;
  bool have_ref_{false};
  bool have_pose_{false};
  bool is_blue_{false};
  double hp_{0.0};
  int stage_{0};
  double x_{0.0};
  double y_{0.0};
  std::size_t target_{0};
  int direction_{1};  // +1 outbound, -1 toward home
};

}  // namespace thornbots_nav
