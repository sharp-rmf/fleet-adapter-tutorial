# Fleet Adapter Transformation Space ( FATS ) 
`git clone --recursive git@github.com:cnboonhan94/fleet-adapter-tutorial.git`
Need ROS1 Melodic and ROS2 Dashing.

## One time: Install dependencies for mir simulation:

```
cd fleet-adapter-tutorial/ros1/src/mir_minimal_example/mir_robot
sudo apt-get update -qq
sudo apt-get install -qq -y python-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths ./ -i -y --rosdistro melodic

```
## Build and Run
Run these! Change "A" to anything from A to H. From the root project folder:
```
cd fleet-adapter-tutorial
tools/build_scripts/build_all.sh
tools/tutorial/04-launch-minimal-mir-full.sh
ros2 topic pub -1 /mir_fleet_manager/waypoint_goal std_msgs/String "data: A"
```

To view the map:
```
tools/traffic-editor/traffic_editor.sh 

```
Then open project -> `maps/maze/maze.yaml`

To generate a nav map from traffic-editor yaml:
```
tools/traffic-editor/generate_nav_map maze
```
