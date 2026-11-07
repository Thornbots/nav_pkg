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

// Publishes the current route waypoint as the MCB's NAV_GOAL; see README.md.

#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "dji_serial_bridge/msg/ref_sys_status.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "thornbots_nav/route_core.hpp"
#include "thornbots_nav/square_core.hpp"

namespace thornbots_nav
{

using dji_serial_bridge::msg::RefSysStatus;
using geometry_msgs::msg::PointStamped;
using nav_msgs::msg::Odometry;

class WaypointNav : public rclcpp::Node
{
public:
  WaypointNav()
  : Node("waypoint_nav")
  {
    // "pipes": the match route below; "square": the off-field test square.
    const auto route = declare_parameter("route", std::string("pipes"));
    if (route == "pipes") {
      core_.emplace(load_params());
    } else if (route == "square") {
      square_.emplace(
        declare_parameter("square_side_m", rclcpp::PARAMETER_DOUBLE).get<double>(),
        declare_parameter("square_tolerance_m", rclcpp::PARAMETER_DOUBLE).get<double>());
    } else {
      throw std::invalid_argument("route must be pipes or square, got " + route);
    }
    // Same frame mcb_relay uses for RELOCALIZE: the MCB's corrected navigation frame.
    frame_id_ = declare_parameter("goal_frame", std::string("nav_odom"));
    const double rate_hz = declare_parameter("rate_hz", 10.0);
    if (!std::isfinite(rate_hz) || rate_hz <= 0.0) {
      throw std::invalid_argument("rate_hz must be finite and > 0");
    }
    // Field-centred nav_odom pose that mcb_relay relocalizes the MCB to; raw /odom
    // boots at (0, 0) at the physical start, so it cannot judge field waypoints.
    const auto odom_topic = declare_parameter("odom_topic", std::string("/localization/odom"));

    pub_ = create_publisher<PointStamped>("/dji_serial_bridge/nav_goal", 10);
    // Lockstep: a Header at each tick that sends no NAV_GOAL (no goal yet).
    const auto none_topic = declare_parameter("nav_goal_none_topic", std::string(""));
    if (!none_topic.empty()) {
      none_pub_ = create_publisher<std_msgs::msg::Header>(none_topic, 10);
    }
    ref_sub_ = create_subscription<RefSysStatus>(
      "/dji_serial_bridge/ref_sys", rclcpp::SensorDataQoS(),
      [this](RefSysStatus::ConstSharedPtr m) {
        if (core_) {
          core_->on_ref_sys(m->is_on_blue_team, m->robot_hp, m->game_stage);
        }
        log_target();
      });
    odom_sub_ = create_subscription<Odometry>(
      odom_topic, 10, [this](Odometry::ConstSharedPtr m) {
        if (core_) {
          core_->on_pose(m->pose.pose.position.x, m->pose.pose.position.y);
        } else {
          square_->on_pose(m->pose.pose.position.x, m->pose.pose.position.y);
        }
        log_target();
      });
    // Lockstep: a best-effort sample sent before a subscription has matched is lost, so
    // the run waits (epoch zero, wall time) until the referee input has a publisher. The
    // localization input appears only once the clock runs, and any later sample serves.
    if (declare_parameter("epoch_zero_ready", false)) {
      ready_pub_ = create_publisher<std_msgs::msg::Header>(
        "~/epoch_zero_ready", rclcpp::QoS(1).reliable().transient_local());
      ready_timer_ = create_wall_timer(
        std::chrono::milliseconds(10), [this]() {
          if (ref_sub_->get_publisher_count() > 0) {
            ready_pub_->publish(std_msgs::msg::Header{});
            ready_timer_->cancel();
          }
        });
    }
    // Node clock, so lockstep sim time paces it like every other input.
    timer_ = create_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate_hz)),
      [this]() {publish();});
  }

private:
  RouteParams load_params()
  {
    // Defaults come from config/route.yaml; an unset parameter throws.
    const auto array = [this](const char * name) {
        return declare_parameter(name, rclcpp::PARAMETER_DOUBLE_ARRAY).get<std::vector<double>>();
      };
    const auto number = [this](const char * name) {
        return declare_parameter(name, rclcpp::PARAMETER_DOUBLE).get<double>();
      };
    RouteParams p;
    p.x = array("waypoints_x");
    p.y = array("waypoints_y");
    p.tolerance_m = array("tolerances_m");
    p.max_hp = number("max_hp");
    p.retreat_hp_ratio = number("retreat_hp_ratio");
    p.advance_hp_ratio = number("advance_hp_ratio");
    p.in_game_stage = static_cast<int>(
      declare_parameter("in_game_stage", rclcpp::PARAMETER_INTEGER).get<int64_t>());
    return p;
  }

  std::optional<Goal> goal() const
  {
    return core_ ? core_->goal() : square_->goal();
  }

  void log_target()
  {
    const auto goal = this->goal();
    if (!goal || (last_index_ && *last_index_ == goal->index)) {
      return;
    }
    last_index_ = goal->index;
    RCLCPP_INFO(
      get_logger(), "Target %zu at (%.3f, %.3f), %s", goal->index, goal->x, goal->y,
      !core_ ? "square" : core_->direction() > 0 ? "outbound" : "returning");
  }

  void publish()
  {
    const auto goal = this->goal();
    if (!goal) {
      if (none_pub_) {
        std_msgs::msg::Header none;
        none.stamp = now();
        none_pub_->publish(none);
      }
      return;
    }
    PointStamped msg;
    msg.header.stamp = now();  // freshly decided
    msg.header.frame_id = frame_id_;
    msg.point.x = goal->x;
    msg.point.y = goal->y;
    msg.point.z = 0.0;
    pub_->publish(msg);
  }

  std::optional<RouteCore> core_;
  std::optional<SquareCore> square_;
  std::string frame_id_;
  std::optional<std::size_t> last_index_;
  rclcpp::Publisher<PointStamped>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr none_pub_;
  rclcpp::Subscription<RefSysStatus>::SharedPtr ref_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_, ready_timer_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr ready_pub_;
};

}  // namespace thornbots_nav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<thornbots_nav::WaypointNav>());
  rclcpp::shutdown();
  return 0;
}
