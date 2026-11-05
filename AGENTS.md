# thornbots_nav

Follow [workspace rules](../AGENTS.md) and [CI](../docs/CI.md).
Read the [README](README.md) for behavior and parameters before changing the route logic.

## Scope

Jetson-side waypoint navigation: walk a fixed route and publish the current
goal as NAV_GOAL. The MCB only follows it; motion control, stuck handling and
spin stay in firmware. Keep runtime nodes C++ with ROS-free cores unit-tested;
Python is for launch. Route defaults live only in [config/route.yaml](config/route.yaml).

## Open

- Package CI runs on `main` only; its pinned workspace `ros.yml` must list
  `nav_pkg` before the first promotion (workspace nightly lists it).
