# contrail
 A collection of nodes and libraries to generate and follow a velocity-dependant path in 3D space.

## Usage
`roslaunch contrail generate.launch path_name:=circle`

## Output
The path output uses the message type `nav_msgs/Path` to define a list of poses for a robot to follow.

Each pose is timestamped relative to the `header.stamp` in the `Path` message. As such, each pose represents where the robot should be at a specific time. Add the pose's stamp to the path's stamp to get the desired pose of the robot in real time.

The difference in pose timestamps represents can be interpolated for greater tracking accuracy, and the difference in pose timestamps can be used to derive the desired velocity (assumes that the velocity is in the direction of the line p(t-1) to p(t).

## Using PathExtract()
The `PathExtract()` library allows you to integrate the trajectory tracking functionallity into your own code.

The main function `get_ref_state()` will extract the current state reference (pose and linear velocity) from a path that has been recieved.

```C++
#include <ros/ros.h>
#include <contrail/path_extract.h>
...
int main(int argc, char** argv) {
...
ros::NodeHandle nhp("~");
contrail(&nhp);
...
while(ros::ok()) {
	bool success = false;

	geometry_msgs::Pose ref_pose;
	geometry_msgs::Vector3 ref_l_vel;

	if(contrail.has_valid_path() || contrail.has_valid_fallback()) {
		success = contrail.get_ref_state(ref_pose, ref_l_vel, ros::Time::now());
	}

	if(success) {
		//Perform some form of control using ref_pose and ref_l_vel
	} else {
		//Perform some form of failsafe, as (most likely) a path has not been sent
	}
	...
}

```

## Generating a Path
A path is generated from parameters (loaded with the launch file from the `.yaml` files) by first setting a starting point, then making lines and arcs from one location to the next. Examples can be found in the `path` folder.

Most parameters will have a default value that will be used if they are omitted from the parameter file for a specific segment.

The parameters are set as follows:

#### start/position (required)
A set of XYZ coordinates for the path to start at.

#### start/alpha
The starting yaw orientation of the path in radians.

Default: 0.0 rad

#### start/velocity
The assumed starting velocity.

Default: 0.0 m/s

## Path segments
The type of path segment generated depends on the parameters set for each segment. There are 3 primary different types of segments:
- Hold
- Line
- Arc

For the hold and line segments, there are also some variations that will occur if the depending on the additional parameters specified. In the most basic case, these segments will be represented by a single pose and time delay.

#### path/sX/present
Must be set to "true" for the segment to be used during generation. Once the programe cannot find any more segments, it stops generating the path.

#### path/sX/hold_time
This causes the robot to be instructed to hold the current location (zero velocity), however, changes in desired heading can still be made (using: `alpha` and `theta`).

#### path/sX/velocity
Sets the desired speed to reach by the end of this segment. If it is the same as the previous segment, the velocity will be kept constant.

Default: [same as previous segment]

#### path/sX/height
A height modifier for the end of the line segment in meters. Setting this to a positive or negative value will make the end location higher or lower than the the starting location respectively.

#### path/sX/length
The forwards facing distance this segment will trace in XY space in meters. Required to define a line segment.

#### radius
The radial distance of a lateral arc to use during segment to complete in radians. If this is set, `alpha` must also be set to determine the arc rotation

#### alpha
The angle in which to turn the path direction during this segment in radians. This parameter can be used to modify both the hold and arc segment types

## The "theta" parameter
Both arc and line segments can have the theta parameter. It is an overide for the desired heading of the robot. This allows for the path to arc or change direction with the robot set to face a different direction.

If the theta parameter is not set, the current heading will be set using the previous robot heading (or alpha for the starting point) to set the new robot heading. This means that if the robot is instructed to complete and arc with no theta set, the robots heading will be rotated relatively to it's starting rotation by the `alpha` parameter of the arc over the entire duration of the arc. Example: If the robot is at -pi/2, and the arc is a rotation of +pi/4, the robot will perform a slerp rotation from -pi/2 to -pi/4 over the duration of the arc.
