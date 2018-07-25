# contrail
A collection of nodes and libraries to generate and track discrete or velocity-dependant trajectories in 3D space.

## Dependencies
The following ROS and system dependencies are required to compile this package:
```sh
sudo apt install ros-kinetic-nav-msgs ros-kinetic-geometry-msgs ros-kinetic-mavros-msgs ros-kinetic-message-generation ros-kinetic-dynamic-reconfigure libeigen3-dev
```

## Basic Functionallity
A selection of nodes have been provided to allow for most users to run directly to get access to the functionallity that contrail offers. Once the package is compiled, each can be run with: `rosrun contrail NODENAME`. Additionally, launch file examples for all nodes have been provided for each node. These nodes are:
- `contrail_guidance_node`:
- `load_waypoints`:
- `load_spline`:
- `converter_waypoints_path`:
- `converter_path_spline`:
- Additionally, a few test scripts are also provided: `test_pose`, `test_path`, and `test_wapoints`.

## Interfacing
As an example of usage, we will look at how contrail is integrated with the `contrail_guidance_node`. When run, this node provides the following interfaces:
- Inputs:
  - `~state/odom`: the current state of the UAV that is to be tracked
- Outputs:
  - `~command/triplet`: the current command that to be sent to the UAV
  - `~feedback/pose`: A visualization aid for the currently commanded pose
  - `~feedback/twist` A visualization aid for the currently commanded velocity

The contrail library adds in the following topic interfaces:
- Inputs:
  - `~reference/contrail/pose`: Sets a discrete pose reference that will be tracked indefinitely
  - `~reference/contrail/path`: Sets a discrete path reference that will be tracked 1 step at a time
  - `~reference/contrail/spline`: A continuous spline reference is generated that will track through each of the points specified
- Outputs:
  - `~feedback/contrail/discrete_progress`: An update on the current progress during discrete tracking modes (output each time the waypoint criteria is satisfied)
  - `~feedback/contrail/spline_approximation`: A path representing the an approximation of the gnerated spline reference (output each time a new spline is generated)
  - `~feedback/contrail/spline_points`: A path representing the points used to perform the spline generation (output each time a new spline is generated)

The discrete tracking works by incrementing the current path step (where a pose is treaded as a 1-step path) once a waypoint criteria is met. The cirteria is defined as follows:
1. The UAV is within a spacial distance defined by the `~/contrail/waypoint_radius` parameter
2. The UAV's yaw rotation is within the margin defined by the `~/contrail/waypoint_yaw_accuracy` parameter
3. The UAV has maintained criteria 1 and 2 for the duration defined by the `~/contrail/waypoint_hold_duration` parameter
Once all the waypoint critera is met, a discrete progress message is output to allow for higher-level interfaces to track progress

Additionally, contrail also adds:
- A service interface to allow switching between different tracking schemes on the fly: `~/contrail/set_tracking`
- A dynamic reconfigure interface to manage parameters: `~/contrail`

Lastly, some additional functionallity can be set via other parameters:
- `contrail/fallback_to_pose`: When set to true, it allows contrail to fallback to holding the last pose used any of the references have been completed. If false, contrail will switch back to having no current reference.
- `contrail/spline_res_per_sec`: Sets how many spline approximation points are used over the duration of the path per second

## Typical Usage
A typical use case of contrail would be to track a pre-plannedd set of discrete waypoints. When a new reference is recieved, contrail will automatically switch to tracking the new reference. However, this does not necessarily mean the previously tracked reference is discarded.



## Developer Functionallity
(Coming soon...)
