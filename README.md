# Fleet Manager
`git clone --recursive git@github.com:cnboonhan94/fleet-adapter-tutorial.git -b 01-fleet-manager`

This chapter will focus on the fleet manager. The role of the fleet manager is to process high level commands, into low level actions that can be carried out by the robot. In some cases, certain vendors will already have written fleet managers for their systems. In our example, this is not the case, so we build a simple one. Recall that our MiR takes in `geometry_msgs/PoseStamped` messages as nav goals. The role of our fleet manager is to manage the delivery, modification and cancellation of such commands, especially in the case of multiple message submissions. In particular, the fleet manager will also expose an interface to ROS2 via bridging in order to communicate with the fleet adapters.

## Building the Fleet Manager
* `cd ros1`
* `colcon build`
* `source install/setup.bash`

## Running the Fleet Manager
`roslaunch mir_fleet_manager fleet_manager.launch`

We should now see an interface very simlar to that in `master` branch. However, we have enhanced capabilities now. In rviz, we can now sequentially place `2D nav goals`, and the fleet manager will execute them in sequence. In addition, if we use `Publish Point`, we can trigger a clear of the current queue of nav goals. 

## Detail
The important topics for the fleet manager are:
* `/mir_fleet_manager/waypoint_goal` : Publish a `PoseStamped` message here to queue a waypoint for the robot.
* `/mir_fleet_manager/reset`: Resets the waypoint queue. I used a PoseStamped message here in order to make use of the existing rviz `Publish Point` button.
* `/mir_fleet_manager/waypoints`: A `PoseArray` of all the waypoints in the queue. Used for visualization.

We can consider the fleet manager to be a high level interface to the robot itself, where we consider destinations in terms of "Waypoints" rather than the specific trajectories the robot will take. This, as we will see, will integrate nicely into the fleet adapters and RMF.

## Bridging to ROS2
It is necessary to bridge the fleet manager from ROS1 to ROS2. This is necessary because RMF runs on ROS2. Thus the vendor navstack will need to expose certain topics to ROS2. We will use [soss](https://github.com/osrf/soss) for this purpose. 

## Installing SOSS
This script will only work for ROS2 Dashing and ROS Melodic. If this fails, follow the installation instructions in the `soss` repo, taking the workspaces specified to be `bridge_tools/ros1` and `bridge_tools/ros2` respectively. Run the following in an unsourced workspace:
* `cd bridge_tools`
* `./install_bridge.sh`

## Testing the Bridge
In the following, a ROS1 terminal is a terminal sourced with ROS1 and sourced with the corresponding `ros1` workspace in the tutorial, vice versa for ROS2.

* In one ROS1 terminal (1): `roslaunch mir_fleet_manager fleet_manager.launch` to run the fleet manager simulation. **Remember** to start the physics in the simulation after everything is online.
* In a clean terminal, run (2): `bridge_tools/run_bridge.sh` from the `fleet-adapter-tutorial` root folder to run the bridging.
* From another ROS2 terminal (3), publish a nav goal as a ROS2 PoseStamped message:
```
ros2 topic pub -r 10 /mir_fleet_manager/waypoint_goal geometry_msgs/PoseStamped '
{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, 
pose: {position: {x: 17.0, y: 10.0, z: 0.0}, 
orientation: {w: 1.0}}}'
```
**This is not working perfectly fine yet, to debug with experts!**

The fleet manager simulation should receive the ROS2 nav goals as if it was published in ROS1. 