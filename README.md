# thornbots_nav

`waypoint_nav` walks a fixed four-point route and publishes the current
waypoint as the MCB's NAV_GOAL. The firmware does the driving. The route logic
is ported from the firmware's former `SimpleAutoDriveCommand`, without its
stuck timer. The firmware's `AutoDriveCommand` holds a field quarter turn while
travelling and spins only at a goal
([MCBV3 README](../firmware/MCBV3/README.md#aim-and-navigation-odometry)).

## Run

`thornbots_pkg`'s `auto.launch.py` starts it with the rest of the robot stack.
Standalone, in a container terminal with `dji_serial_bridge` and localization running:

```bash
colcon build --packages-select thornbots_nav
source install/setup.bash
ros2 launch thornbots_nav waypoint_nav.launch.py
ros2 topic echo /dji_serial_bridge/nav_goal
```

Launch arguments: `params_file` (default `config/route.yaml`), `route`
(`pipes`, the default, or `square`) and `use_sim_time` (default `false`).
From the robot stack, pass `nav_route:=square` to `auto.launch.py`.

## Topics

| Topic | Type | QoS | Direction |
| --- | --- | --- | --- |
| `/dji_serial_bridge/ref_sys` | `dji_serial_bridge/msg/RefSysStatus` | SensorData | in: `game_stage`, `robot_hp`, `is_on_blue_team` |
| `/localization/odom` | `nav_msgs/msg/Odometry` | reliable, depth 10 | in: `pose.position` x/y |
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

## Square test route

`route:=square` is for testing off the field. The first `/localization/odom`
pose is corner 0, and the robot loops +1 m x, then +1 m y, then back, then
home (`square_side_m`). The square is not mirrored for blue, and it ignores
REF_SYS (HP, game stage, team), so it runs with no referee. A corner counts as
reached within `square_tolerance_m` (0.2 m). That is outside the firmware's
0.15 m spin radius, so the robot moves to the next corner without stopping to
spin. Under RMUL 3v3 referee data the firmware still holds the robot before the
countdown.

## Parameters

[config/route.yaml](config/route.yaml) is the source of defaults and is
required: `waypoints_x`, `waypoints_y`, `tolerances_m`, `max_hp`,
`retreat_hp_ratio`, `advance_hp_ratio`, `in_game_stage`. `rate_hz` (default 10)
is also set there. `nav_goal_none_topic` (default empty; the lockstep simulator sets it)
publishes a `Header` on each tick that sends no NAV_GOAL. Startup throws unless the arrays have equal length ≥ 2, all
values are finite, tolerances and `max_hp` are > 0 and
0 < `retreat_hp_ratio` < `advance_hp_ratio` ≤ 1. `max_hp` is 400 because REF_SYS
carries no maximum ([sentry context](../docs/ARCC_2026_SENTRY_CONTEXT.md)).

Code-level options: `odom_topic` (default `/localization/odom`) and
`goal_frame` (default `nav_odom`). `epoch_zero_ready` (default false; the lockstep simulator
sets it) publishes `~/epoch_zero_ready` once the referee subscription has a publisher: a
best-effort sample sent before the subscription matches is lost, and the goal then starts a tick
later in that run.

## Frames

Waypoints are field-centred metres, and NAV_GOAL is in `nav_odom`
([wire frames](../ros2_dji_serial_bridge/UART_PROTOCOL.md#message-types)).
Arrival uses `/localization/odom`, the field-centred `nav_odom` pose that
`mcb_relay` relocalizes the MCB to, so the Jetson and the MCB judge goals in
the same frame. Raw `/odom` boots at (0, 0) at the physical start and cannot.

## Tests

```bash
colcon test --packages-select thornbots_nav && colcon test-result --verbose
```

`test_route_core` and `test_square_core` cover the route logic without ROS; the pytest suite lints the
launch file.
