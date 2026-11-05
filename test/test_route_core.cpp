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

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "thornbots_nav/route_core.hpp"

namespace thornbots_nav
{
namespace
{

constexpr int kInGame = 4;
constexpr int kPreGame = 2;
constexpr double kFull = 400.0;
constexpr double kBand = 280.0;  // between retreat (221) and advance (396)
constexpr double kLow = 200.0;

// Same values as config/route.yaml.
RouteParams route_params()
{
  RouteParams p;
  p.x = {-5.25, -5.25, -2.40, 0.00};
  p.y = {-3.00, -0.635, -0.635, -2.40};
  p.tolerance_m = {0.5, 0.05, 0.2, 0.5};
  p.max_hp = 400.0;
  p.retreat_hp_ratio = 0.5525;
  p.advance_hp_ratio = 0.99;
  p.in_game_stage = kInGame;
  return p;
}

// Delivers a pose and a REF_SYS, as the node's two callbacks would. The pose
// goes first so the new HP is judged against the new position, not a stale one.
void feed(
  RouteCore & core, double x, double y, double hp = kFull, int stage = kInGame,
  bool blue = false)
{
  core.on_pose(x, y);
  core.on_ref_sys(blue, hp, stage);
}

std::size_t index_of(const RouteCore & core)
{
  return core.goal().value().index;
}

// Drives a red robot in game along the route up to waypoint `index`.
void drive_out_to(RouteCore & core, std::size_t index)
{
  const auto p = route_params();
  for (std::size_t i = 0; i <= index; ++i) {
    feed(core, p.x[i], p.y[i]);
  }
}

TEST(RouteCore, NoGoalUntilBothInputsArrive)
{
  RouteCore core(route_params());
  EXPECT_FALSE(core.goal());
  core.on_ref_sys(false, kFull, kInGame);
  EXPECT_FALSE(core.goal());

  RouteCore other(route_params());
  other.on_pose(-5.25, -3.0);
  EXPECT_FALSE(other.goal());
  other.on_ref_sys(false, kFull, kInGame);
  EXPECT_TRUE(other.goal());
}

TEST(RouteCore, StartsAtHomeOutbound)
{
  RouteCore core(route_params());
  feed(core, 0.0, 0.0);  // far from H
  const auto goal = core.goal().value();
  EXPECT_EQ(goal.index, 0u);
  EXPECT_DOUBLE_EQ(goal.x, -5.25);
  EXPECT_DOUBLE_EQ(goal.y, -3.0);
  EXPECT_EQ(core.direction(), 1);
}

TEST(RouteCore, RedOutboundUsesPerWaypointTolerances)
{
  RouteCore core(route_params());
  feed(core, -4.80, -3.0);  // 0.45 m from H, inside 0.5
  EXPECT_EQ(index_of(core), 1u);

  feed(core, -5.25, -0.635 - 0.06);  // doorway centreline needs 0.05
  EXPECT_EQ(index_of(core), 1u);
  feed(core, -5.25, -0.635 - 0.04);
  EXPECT_EQ(index_of(core), 2u);

  feed(core, -2.40 + 0.21, -0.635);  // doorway needs 0.2
  EXPECT_EQ(index_of(core), 2u);
  feed(core, -2.40 + 0.19, -0.635);
  EXPECT_EQ(index_of(core), 3u);

  const auto goal = core.goal().value();
  EXPECT_DOUBLE_EQ(goal.x, 0.0);
  EXPECT_DOUBLE_EQ(goal.y, -2.4);
}

TEST(RouteCore, AdvancesOneStepPerUpdate)
{
  RouteCore core(route_params());
  feed(core, -5.25, -3.0);
  EXPECT_EQ(index_of(core), 1u);  // not 2, although H and 1 are 2.4 m apart
}

TEST(RouteCore, BlueMirrorsX)
{
  RouteCore core(route_params());
  feed(core, 5.25, -3.0, kFull, kInGame, true);
  EXPECT_EQ(index_of(core), 1u);
  EXPECT_DOUBLE_EQ(core.goal()->x, 5.25);
  EXPECT_DOUBLE_EQ(core.goal()->y, -0.635);
  feed(core, 5.25, -0.635, kFull, kInGame, true);
  feed(core, 2.40, -0.635, kFull, kInGame, true);
  EXPECT_EQ(index_of(core), 3u);
  EXPECT_DOUBLE_EQ(core.goal()->x, 0.0);

  // Red coordinates do not count as arrival for blue.
  RouteCore red_pose(route_params());
  feed(red_pose, -5.25, -3.0, kFull, kInGame, true);
  EXPECT_EQ(index_of(red_pose), 0u);
}

TEST(RouteCore, HoldsAtPipesPoint)
{
  RouteCore core(route_params());
  drive_out_to(core, 3);
  EXPECT_EQ(index_of(core), 3u);
  feed(core, 0.0, -2.4);
  feed(core, 0.0, -2.4);
  EXPECT_EQ(index_of(core), 3u);
  EXPECT_EQ(core.direction(), 1);
}

TEST(RouteCore, PreGameHoldsOutboundAdvancing)
{
  RouteCore core(route_params());
  feed(core, -5.25, -3.0, kFull, kPreGame);
  EXPECT_EQ(index_of(core), 0u);
  feed(core, -5.25, -3.0, kFull, kInGame);
  EXPECT_EQ(index_of(core), 1u);
}

TEST(RouteCore, ReturningHomeIsAllowedInAnyStage)
{
  RouteCore core(route_params());
  drive_out_to(core, 3);
  feed(core, 0.0, -2.4, kLow, kPreGame);  // retreat: 3 -> 2
  EXPECT_EQ(index_of(core), 2u);
  feed(core, -2.4, -0.635, kLow, kPreGame);
  EXPECT_EQ(index_of(core), 1u);
  feed(core, -5.25, -0.635, kLow, kPreGame);
  EXPECT_EQ(index_of(core), 0u);
}

TEST(RouteCore, HeadingOutAgainWaitsForTheGame)
{
  RouteCore core(route_params());
  drive_out_to(core, 1);
  feed(core, -5.25, -0.635, kLow, kInGame);  // retreat, arrive at 1 -> 0
  feed(core, -5.25, -3.0, kLow, kInGame);
  EXPECT_EQ(index_of(core), 0u);
  feed(core, -5.25, -3.0, kFull, kPreGame);  // healed, but not in game
  EXPECT_EQ(index_of(core), 0u);
  EXPECT_EQ(core.direction(), -1);
  feed(core, -5.25, -3.0, kFull, kInGame);
  EXPECT_EQ(core.direction(), 1);
  EXPECT_EQ(index_of(core), 1u);
}

TEST(RouteCore, HysteresisBandKeepsDirection)
{
  RouteCore core(route_params());
  feed(core, 0.0, 0.0, kBand);  // outbound stays outbound
  EXPECT_EQ(core.direction(), 1);

  feed(core, 0.0, 0.0, kLow);
  EXPECT_EQ(core.direction(), -1);
  feed(core, 0.0, 0.0, kBand);  // returning stays returning
  EXPECT_EQ(core.direction(), -1);
  feed(core, 0.0, 0.0, 390.0);  // still <= 0.99 * 400
  EXPECT_EQ(core.direction(), -1);
  feed(core, 0.0, 0.0, 397.0);
  EXPECT_EQ(core.direction(), 1);
}

TEST(RouteCore, RetreatFromPipesPointRetracesThePath)
{
  RouteCore core(route_params());
  drive_out_to(core, 3);
  feed(core, 0.0, -2.4, kLow);
  EXPECT_EQ(core.direction(), -1);
  EXPECT_EQ(index_of(core), 2u);
  EXPECT_DOUBLE_EQ(core.goal()->x, -2.40);

  feed(core, -2.40, -0.635, kLow);
  EXPECT_EQ(index_of(core), 1u);
  feed(core, -5.25, -0.635, kLow);
  EXPECT_EQ(index_of(core), 0u);
  EXPECT_DOUBLE_EQ(core.goal()->y, -3.0);

  feed(core, -5.25, -3.0, kLow);  // stays home
  EXPECT_EQ(index_of(core), 0u);
}

TEST(RouteCore, RetreatMidLegGoesBackToTheWaypointJustLeft)
{
  RouteCore core(route_params());
  drive_out_to(core, 1);  // now heading for 2
  ASSERT_EQ(index_of(core), 2u);
  feed(core, -4.0, -0.635, kLow);  // mid-leg, hurt
  EXPECT_EQ(index_of(core), 1u);  // not 0: no cutting the corner
  EXPECT_EQ(core.direction(), -1);

  feed(core, -4.0, -0.635, kFull);  // healed mid-leg: resume toward 2
  EXPECT_EQ(core.direction(), 1);
  EXPECT_EQ(index_of(core), 2u);
}

TEST(RouteCore, RetreatBeforeLeavingHomeStaysHome)
{
  RouteCore core(route_params());
  feed(core, 0.0, 0.0, kLow);
  EXPECT_EQ(index_of(core), 0u);
  EXPECT_EQ(core.direction(), -1);
}

TEST(RouteParamsValidation, AcceptsTheRoute)
{
  EXPECT_NO_THROW(validate(route_params()));
  EXPECT_NO_THROW(RouteCore{route_params()});
}

TEST(RouteParamsValidation, RejectsBadArrays)
{
  auto p = route_params();
  p.y.pop_back();
  EXPECT_THROW(validate(p), std::invalid_argument);

  p = route_params();
  p.tolerance_m.push_back(0.5);
  EXPECT_THROW(validate(p), std::invalid_argument);

  p = route_params();
  p.x = {0.0};
  p.y = {0.0};
  p.tolerance_m = {0.5};
  EXPECT_THROW(validate(p), std::invalid_argument);

  p = route_params();
  p.x.clear();
  p.y.clear();
  p.tolerance_m.clear();
  EXPECT_THROW(validate(p), std::invalid_argument);
}

TEST(RouteParamsValidation, RejectsNonFiniteValues)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  auto p = route_params();
  p.x[1] = nan;
  EXPECT_THROW(validate(p), std::invalid_argument);
  p = route_params();
  p.y[2] = inf;
  EXPECT_THROW(validate(p), std::invalid_argument);
  p = route_params();
  p.tolerance_m[0] = nan;
  EXPECT_THROW(validate(p), std::invalid_argument);
  p = route_params();
  p.max_hp = inf;
  EXPECT_THROW(validate(p), std::invalid_argument);
  p = route_params();
  p.retreat_hp_ratio = nan;
  EXPECT_THROW(validate(p), std::invalid_argument);
  p = route_params();
  p.advance_hp_ratio = nan;
  EXPECT_THROW(validate(p), std::invalid_argument);
}

TEST(RouteParamsValidation, RejectsNonPositiveTolerancesAndHp)
{
  auto p = route_params();
  p.tolerance_m[1] = 0.0;
  EXPECT_THROW(validate(p), std::invalid_argument);
  p = route_params();
  p.tolerance_m[3] = -0.5;
  EXPECT_THROW(validate(p), std::invalid_argument);
  p = route_params();
  p.max_hp = 0.0;
  EXPECT_THROW(validate(p), std::invalid_argument);
}

TEST(RouteParamsValidation, RejectsBadHpRatios)
{
  auto p = route_params();
  p.retreat_hp_ratio = 0.0;
  EXPECT_THROW(validate(p), std::invalid_argument);
  p = route_params();
  p.retreat_hp_ratio = 0.99;  // not below advance
  EXPECT_THROW(validate(p), std::invalid_argument);
  p = route_params();
  p.retreat_hp_ratio = 0.995;
  EXPECT_THROW(validate(p), std::invalid_argument);
  p = route_params();
  p.advance_hp_ratio = 1.01;
  EXPECT_THROW(validate(p), std::invalid_argument);
  p = route_params();
  p.advance_hp_ratio = 1.0;  // allowed: <= 1
  EXPECT_NO_THROW(validate(p));
}

TEST(RouteParamsValidation, ConstructorThrowsOnBadInput)
{
  auto p = route_params();
  p.tolerance_m[0] = 0.0;
  EXPECT_THROW(RouteCore{p}, std::invalid_argument);
}

}  // namespace
}  // namespace thornbots_nav
