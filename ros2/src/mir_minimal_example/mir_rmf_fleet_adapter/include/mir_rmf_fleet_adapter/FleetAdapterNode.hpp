#ifndef SRC__MIR_RMF_FLEET_ADAPTER__FLEETADAPTERNODE_HPP
#define SRC__MIR_RMF_FLEET_ADAPTER__FLEETADAPTERNODE_HPP

#include <rmf_traffic_ros2/schedule/MirrorManager.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <rmf_traffic_msgs/srv/submit_trajectory.hpp>
#include <std_msgs/msg/string.hpp>

#include <rmf_traffic/agv/Graph.hpp>
#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/agv/VehicleTraits.hpp>

#include <rclcpp/node.hpp>

namespace mir_rmf_fleet_adapter
{

//==============================================================================
class FleetAdapterNode : public rclcpp::Node
{
public:
  // Use this function to initialize Fleet Adapter and get a pointer to it in main().
  static std::shared_ptr<FleetAdapterNode> make(
      std::string fleet_name,
      const std::string &graph_file,
      rmf_traffic::agv::VehicleTraits vehicle_traits,
      rmf_traffic::Duration wait_time = std::chrono::seconds(10));

private:
  // These aliases are used by the adapter to communicate with the scheduler using service calls
  using SubmitTrajectory = rmf_traffic_msgs::srv::SubmitTrajectory;
  using SubmitTrajectoryClient = rclcpp::Client<SubmitTrajectory>;
  using SubmitTrajectoryHandle = SubmitTrajectoryClient::SharedPtr;

  // These hold the initialization contents after running ::make.
  struct Data
  {
    rmf_traffic::agv::Graph graph;                              // RMF representation of the traffic system
    std::unordered_map<std::string, std::size_t> waypoint_keys; // Names of the waypoints ( vertices )
    rmf_traffic::agv::VehicleTraits traits;                     // Relevant description of the robot
    rmf_traffic_ros2::schedule::MirrorManager mirror;           // Local cache of the Schedule Database
    SubmitTrajectoryHandle submit_trajectory;                   // Handle for submitting trajectories to scheduler
  };

  // Constructor: This is a placeholder to allow proper initialization of MirrorManager. ::make must be called to fully initialize this node
  // Thus, use ::make for construction instead
  FleetAdapterNode(std::string fleet_name);

  // This function is called once the node is constructed and mirror is ready, for initialization. In other words ::make = constructor + ::start
  void start(Data data);

  // Duh
  std::string fleet_name;

  // TODO(MXG): Replace this with a std::optional as soon as we can use C++17
  std::unique_ptr<Data> data;

  // Some external system ( Perhaps the upcoming RMF Dispatch Planner ) will publish task requests on this topic.
  using MiRTaskRequest = std_msgs::msg::String;
  using MiRTaskRequestSub = rclcpp::Subscription<MiRTaskRequest>;   // The fleet adapter will implement logic in order to fulfill this task.
  using MirTaskRequestRetryPub = rclcpp::Publisher<MiRTaskRequest>; // Used by this adapter implementation to trigger a local task re-request

  MiRTaskRequestSub::SharedPtr move_robot_sub; // In our example, our tasks are simply to move from point to point, but this is not a hard rule.
  MirTaskRequestRetryPub::SharedPtr move_robot_retry;

  // THE FOLLOWING FUNCTIONS DESCRIBE THE FLEET ADAPTER PROCESS FLOW //
  // First, an external system publishes a TaskRequest to a relevant subscriver
  // handle_task_request will process this request and generate an RMF trajectory.
  // Then, it will submit the trajectory asynchronously using a SubmitTrajectoryClient, passing handle_scheduler_response as a callback function
  void handle_task_request(MiRTaskRequest::UniquePtr msg);

  // When the scheduler decides your fate, it will call the handler_scheduler_response callback.
  // This function will handle the logic regarding an accepted or requested trajectory submission.
  void handle_scheduler_response(const SubmitTrajectoryClient::SharedFuture &response);

  // In the case of a successful submission, the fleet adapter will now turn to the fleet manager and give instructions.
  using FleetManagerPub = rclcpp::Publisher<geometry_msgs::msg::PoseStamped>;
  FleetManagerPub::SharedPtr fleet_manager_publisher;
  
  // This function handles the conversion of an accepted trajectory into fleet_manager compatible messages. 
  void move_fleet(rmf_traffic::Trajectory trajectory);
};

} // namespace mir_rmf_fleet_adapter

#endif // SRC__MIR_RMF_FLEET_ADAPTER__FLEETADAPTERNODE_HPP