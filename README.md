# Fleet Adapter Transformation Space ( FATS )
Run these! Change "A" to anything from A to H.
```
git clone --recursive git@github.com:cnboonhan94/fleet-adapter-tutorial.git
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
tools/traffic-editor generate_nav_map maze
```
