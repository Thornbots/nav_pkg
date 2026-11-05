# thornbots_nav

Follow [workspace rules](../AGENTS.md) and [CI](../docs/CI.md).
Read the [README](README.md) for behavior and parameters before changing the route logic.

## Scope

Jetson-side waypoint navigation: walk a fixed route and publish the current
goal as NAV_GOAL. The MCB only follows it; motion control, stuck handling and
spin stay in firmware. Keep runtime nodes C++ with ROS-free cores unit-tested;
Python is for launch. Route defaults live only in [config/route.yaml](config/route.yaml).

## Open

- `ros.yml` in the workspace does not list `nav_pkg` as a portable package yet, so
  package CI is rejected until it does.
- Arrival is measured on `/odom`; confirm it is field-centred
  ([README](README.md#frames)).
