# RMF Schedule ROS2 Node ( Currently doing work, might break stuff )
`git clone --recursive git@github.com:cnboonhan94/fleet-adapter-tutorial.git -b 04-schedule-node`

This chapter introduces the ROS2 RMF Schedule Node. This is an implementation of the RMF Schedule as described in [03-rmf-core-overview](https://github.com/cnboonhan94/fleet-adapter-tutorial/tree/03-rmf-core-overview). For this chapter, the Schedule will be empty. Following this, a fleet adapter ( implemented uniquely for each individual fleet ) will use the Schedule to determine feasible trajectories. It will then instruct the **fleet manager** on the high level path to take, and update the **Schedule** through ROS2 *service calls*.

## Building the Schedule Node
* Source your ROS2 Distro
* `cd ros2`
* `colcon build --packages-up-to rmf_traffic_schedule`
* `source install/setup.bash`
* `ros2 run rmf_traffic_schedule run_schedule`

Now, open another ROS2 terminal and `ros2 topic list`. You will see the new topic:
```
/rmf_traffic/mirror_wakeup
```
Now run `ros2 service list`. You will see the following services:
```
/rmf_traffic/erase_schedule
/rmf_traffic/mirror_update
/rmf_traffic/register_query
/rmf_traffic/submit_trajectory
/rmf_traffic/unregister_query
/rmf_traffic_schedule_node/describe_parameters
/rmf_traffic_schedule_node/get_parameter_types
/rmf_traffic_schedule_node/get_parameters
/rmf_traffic_schedule_node/list_parameters
/rmf_traffic_schedule_node/set_parameters
/rmf_traffic_schedule_node/set_parameters_atomically
```

## Understanding the Schedule Node
Recall that the Schedule captures information about other fleets in the environment as a Database of Trajectories. Also recall that there are Mirrors, which act as local copies of the main Schedule's Database for each of the fleet adapters. Now, lets analyse the above services and topics.

### /rmf_traffic/mirror_wakeup
This is the only topic publishing [MirrorWakeup.msg](https://github.com/osrf/rmf_core/blob/traffic_msgs/rmf_traffic_msgs/msg/MirrorWakeup.msg). The Schedule will periodically publish MirrorWakeup.msg to downstream Mirrors, which will then trigger a `mirror_update` service.

### Services
Reading the service names should give a good idea of what they do. Refer [here](https://github.com/osrf/rmf_core/tree/traffic_msgs/rmf_traffic_msgs/srv) for service message details, APIs and what to expect as responses.
