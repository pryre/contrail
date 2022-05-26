# contrail
A collection of nodes and libraries to generate and track discrete or velocity-dependant trajectories in 3D space.

## Contrail Software
Further details on the software side of this package can be found in the [contrail readme](contrail_manager/README.md)

## Contrail Messages
The `contrail_msgs` package defines a set of messages to allow for basic high-level interaction with the contrail library.

## GUI Displays

#### RQT Contrail Planner
Ground Control Station plugin for RQT to allow easy planning of flight trajectories.

![Screenshot](/rqt_contrail_planner/resource/screenshot.png)

#### RQT Contrail Commander
An RQT interface for issuing goto-type commands to contrail.

![Screenshot](/rqt_contrail_commander/resource/screenshot.png)


## External Libraries
The [TinySpline](https://github.com/msteinbeck/tinyspline) library is utilized to perform some of the tracking functionallities of this package.
