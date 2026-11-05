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

#include <limits>
#include <stdexcept>

#include "thornbots_nav/square_core.hpp"

namespace thornbots_nav
{
namespace
{

void expect_goal(const SquareCore & core, double x, double y, std::size_t index)
{
  const auto g = core.goal();
  ASSERT_TRUE(g.has_value());
  EXPECT_NEAR(g->x, x, 1e-9);
  EXPECT_NEAR(g->y, y, 1e-9);
  EXPECT_EQ(g->index, index);
}

TEST(SquareCore, NoGoalBeforeTheFirstPose)
{
  SquareCore core(1.0, 0.2);
  EXPECT_FALSE(core.goal().has_value());
}

TEST(SquareCore, LoopsPlusXThenPlusYFromTheFirstPose)
{
  SquareCore core(1.0, 0.2);
  core.on_pose(2.0, -3.0);  // anchors corner 0; first target is +x
  expect_goal(core, 3.0, -3.0, 1);
  core.on_pose(2.5, -3.0);  // half way: not there yet
  expect_goal(core, 3.0, -3.0, 1);
  core.on_pose(2.9, -3.0);
  expect_goal(core, 3.0, -2.0, 2);
  core.on_pose(3.0, -2.1);
  expect_goal(core, 2.0, -2.0, 3);
  core.on_pose(2.1, -2.0);
  expect_goal(core, 2.0, -3.0, 0);
  core.on_pose(2.0, -2.9);  // back home: loop again
  expect_goal(core, 3.0, -3.0, 1);
}

TEST(SquareCore, LaterPosesNeverMoveTheAnchor)
{
  SquareCore core(1.0, 0.2);
  core.on_pose(0.0, 0.0);
  core.on_pose(5.0, 5.0);  // far from everything
  expect_goal(core, 1.0, 0.0, 1);
}

TEST(SquareCore, ToleranceIsInclusiveAndSideScales)
{
  SquareCore core(2.0, 0.25);
  core.on_pose(0.0, 0.0);
  core.on_pose(1.75, 0.0);  // exactly 0.25 m short
  expect_goal(core, 2.0, 2.0, 2);
}

TEST(SquareCore, RejectsBadParameters)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(SquareCore(0.0, 0.1), std::invalid_argument);
  EXPECT_THROW(SquareCore(-1.0, 0.1), std::invalid_argument);
  EXPECT_THROW(SquareCore(nan, 0.1), std::invalid_argument);
  EXPECT_THROW(SquareCore(1.0, 0.0), std::invalid_argument);
  EXPECT_THROW(SquareCore(1.0, 0.5), std::invalid_argument);
  EXPECT_THROW(SquareCore(1.0, nan), std::invalid_argument);
  EXPECT_NO_THROW(SquareCore(1.0, 0.2));
}

}  // namespace
}  // namespace thornbots_nav
