# Waypoint Mapping
`git clone --recursive git@github.com:cnboonhan94/fleet-adapter-tutorial.git -b 02-waypoint-mapping`

We initially assume that the vendor setup comes with a complete navigation stack. This includes a navigation map. We can find the map for this example in `ros1/src/mir_robot/mir_gazebo/maps/maze.png`. The next step would be to annotate this map with waypoints and lanes, which represent the "traffic roads" on which our robot(s) will operate. 

Note that this step is done once per "area of operation", regardless of the number of fleets operating. ( IE O(1) with respect to n fleet adapters ). We will use [traffic-editor](https://github.com/osrf/traffic-editor.git) for this job. We also refactor the directory space for external modules, which are not ROS packages, by adding the `external` folder.

## Building traffic-editor
* `sudo apt update`
* `sudo apt install git cmake libyaml-cpp-dev qt5-default`
* `cd external && mkdir build && cd build`
* `cmake ..`
* `make -j4`

This should be sufficient to build the `traffic-editor` locally in the `external` folder. The executable is found in `/external/build/src/traffic-editor/traffic-editor`.

## Running traffic-editor
The following steps may be necessary until `traffic-editor` is more developed.
* `cd` to root folder
* `external/build/src/traffic-editor/traffic-editor`. Run the executable from the root folder
* Select file -> Open Project, and select `maze.yaml` from `maps/maze-new` or `maps/maze-complete`.

**Important** to note that the executable should be run from root folder, and the path specified in `maze.yaml` must point correctly to `maze.png` relative to the root folder. Alternatively, you can run the executable from `traffic-editor` root folder, and this will allow more features, but the paths specified in `maze.yaml` must then also be changed.

## Introduction
The `traffic-editor`is a top-down editor that allows the annotation of maps. There are two functions of the editor. The first function is to specify the "traffic lanes", which are shared amongst all fleets operating in this area. The second function is to allow the automatic generation of a gazebo world by transforming annotations into their corresponding (3D) gazebo counterparts. This is useful when we do not have a gazebo world of the are, but we have a 2D planar map.

We will focus on the first function in this tutorial, since we conviniently already have a world to work with. Start off with opening `maps/maze-new/maze.yaml` using `traffic-editor`:

```
Select file -> Open Project, and select `maze.yaml` from `maps/maze-new`. 
```
You should see a blank map loaded.

## Understanding the map
It is important to understand the vendor map. For `maze.png`, we can see that the image is 400x400 pixels. In addition, we know from `maze.yaml` in `mir_gazebo/maps` that the resolution is 0.05. Finally, the map is used without any transformations. Thus the 2d coordinate system of this map is taken with reference to the bottom left ( as viewed in rviz ) as the origin (0, 0) and the frame extending 400 x 0.05 = 20 units in the x and y axis.

## Scaling the map
We first set the map scale using the measurement tool. 
* On the top left, select `add vertex` and click on the four corners of the map.
* Select `add measurements` and drag between each vertex to create an edge for each of the four corners of the map.
* On each edge, select `select` and click on the edge. Then, on the top right, a table should appear. Change the `distance` property to 20, which is the true distance given our known map resolution.

## Naming vertices
We can use `select` and click on vertices, and change their `name` property. In `maze`, our convention is to label each of the four corner vertices as "top_left", "bottom_left", "top_right" and "bottom_right" respectively.

## Adding Traffic Lanes
Use `add vertex` to add more vertices within the map boundaries. Then use `add lane` and drag between two adjacent vertices to add a "road" between them.  Use `select` on each lane and set the `bidirectional` property as "true" if this lane should be so. In `maze`, all lanes are bidirectional. Also set the `name` property to a unique identifier. In `maze`, our convention is to label the bottom left as "A", and then continue in alphabetical order clockwise from "A".

## Output
The result of all this should be a populated `maze.yaml` file. This is different from the file in `mir_gazebo`. This file will be used to create RMF Graphs.

## Final Step
Finally, we process the output yaml file, which is too descriptive, into a reduced form for navigation purposes in RMF.
```
cd maps/maze-complete
../../external/src/traffic-editor/generators/nav_generator.py maze.yaml nav
```
