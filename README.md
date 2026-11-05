# thornbots_nav

`waypoint_nav` walks a fixed four-point route and publishes the current
waypoint as the MCB's NAV_GOAL. The firmware does the driving. The route logic
is ported from the firmware's `SimpleAutoDriveCommand`, without its stuck timer
or spin (the firmware keeps both).

## Run

In a container terminal, with `dji_serial_bridge` and `pose_translator` running:

```bash
colcon build --packages-select thornbots_nav
source install/setup.bash
ros2 launch thornbots_nav waypoint_nav.launch.py
ros2 topic echo /dji_serial_bridge/nav_goal
```

Launch arguments: `params_file` (default `config/route.yaml`) and
`use_sim_time` (default `false`).

## Topics

| Topic | Type | QoS | Direction |
| --- | --- | --- | --- |
| `/dji_serial_bridge/ref_sys` | `dji_serial_bridge/msg/RefSysStatus` | SensorData | in: `game_stage`, `robot_hp`, `is_on_blue_team` |
| `/odom` | `nav_msgs/msg/Odometry` | reliable, depth 10 | in: `pose.position` x/y |
| `/dji_serial_bridge/nav_goal` | `geometry_msgs/msg/PointStamped` | reliable, depth 10 | out, at `rate_hz` |

Nothing is published until both a REF_SYS and an odometry message have arrived
(the team is unknown before then). The goal's `header.stamp` is `now()`, `z` is 0
and `frame_id` is `nav_odom`, the frame `mcb_relay` uses for RELOCALIZE and the
[wire owner](../ros2_dji_serial_bridge/UART_PROTOCOL.md#nav_goal-id0--navigation-goal)
defines for NAV_GOAL.

## Behavior

- Start at waypoint 0, outbound (+1).
- HP above `advance_hp_ratio * max_hp` selects outbound; at or below
  `retreat_hp_ratio * max_hp` selects returning; in between keeps the current
  direction (hysteresis).
- Within `tolerances_m[i]` of the current waypoint, step one waypoint in the
  current direction. The ends hold. Outbound steps (and flipping back to
  outbound) wait for `game_stage == in_game_stage`; returning works in any stage.
- A direction flip targets the next waypoint in the new direction from the
  current leg, so the robot re-traces the route and never cuts a corner:
  from P it goes 2 → 1 → H; mid-leg toward 2 it goes back to 1.
- Target changes are logged at INFO.

Blue mirrors the red route (`x → −x`).

## Red route

| Index | Point | x (m) | y (m) | Tolerance (m) |
| --- | --- | --- | --- | --- |
| 0 | H: start/resupply, Starting Zone centre | −5.25 | −3.00 | 0.5 |
| 1 | North out of the zone, on the doorway centreline | −5.25 | −0.635 | 0.05 |
| 2 | East through the 0.745 m doorway | −2.40 | −0.635 | 0.2 |
| 3 | P: pipes point, +y of the bucket at (0, −3.0) | 0.00 | −2.40 | 0.5 |

## Parameters

[config/route.yaml](config/route.yaml) is the source of defaults and is
required: `waypoints_x`, `waypoints_y`, `tolerances_m`, `max_hp`,
`retreat_hp_ratio`, `advance_hp_ratio`, `in_game_stage`. `rate_hz` (default 10)
is also set there. Startup throws unless the arrays have equal length ≥ 2, all
values are finite, tolerances and `max_hp` are > 0 and
0 < `retreat_hp_ratio` < `advance_hp_ratio` ≤ 1. `max_hp` is 400 because REF_SYS
carries no maximum ([sentry context](../docs/ARCC_2026_SENTRY_CONTEXT.md)).

Code-level options: `odom_topic` (default `/odom`) and `goal_frame` (default
`nav_odom`).

## Frames

Waypoints are field-centred metres. NAV_GOAL is in `nav_odom`. Raw `/odom`
boots at (0, 0) at the physical start ([wire frames](../ros2_dji_serial_bridge/UART_PROTOCOL.md#message-types)),
so arrival against these waypoints is only right if `/odom` is field-centred;
otherwise point `odom_topic` at a field-centred source.

## Tests

```bash
colcon test --packages-select thornbots_nav && colcon test-result --verbose
```

`test_route_core` covers the route logic without ROS; the pytest suite lints the
launch file.
