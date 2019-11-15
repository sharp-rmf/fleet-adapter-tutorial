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

namespace mir_rmf_fleet_adapter {

//==============================================================================
class FleetAdapterNode : public rclcpp::Node
{
public:
  // Use this function to initialize Fleet Adapter and get a pointer to it in main().
  static std::shared_ptr<FleetAdapterNode> make(
      std::string fleet_name,
      const std::string& graph_file,
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
    rmf_traffic::agv::Graph graph; // RMF representation of the traffic system
    std::unordered_map<std::string, std::size_t> waypoint_keys; // Names of the waypoints ( vertices )
    rmf_traffic::agv::VehicleTraits traits; // Relevant description of the robot
    rmf_traffic_ros2::schedule::MirrorManager mirror; // Local cache of the Schedule Database
    SubmitTrajectoryHandle submit_trajectory; // Handle for submitting trajectories to scheduler
  };
  
  // Constructor: This is a placeholder to allow proper initialization of MirrorManager. ::make must be called to fully initialize this node
  // Thus, use ::make for construction instead
  FleetAdapterNode(std::string fleet_name);

  // This function is called once the node is constructed and mirror is ready, for initialization. IE ::make = constructor + start
  void start(Data data);

  std::string fleet_name;

  // TODO(MXG): Replace this with a std::optional as soon as we can use C++17
  std::unique_ptr<Data> data;

  // These aliases are used by some external system to send requests of action to the fleet adapter (TODO: Migrate these to external node)
  using MiRTaskRequest = std_msgs::msg::String;
  using MiRTaskRequestSub = rclcpp::Subscription<MiRTaskRequest>;
  using MirTaskRequestRetryPub = rclcpp::Publisher<MiRTaskRequest>; // Used by this adapter implementation to trigger a local task re-request 

  MiRTaskRequestSub::SharedPtr move_robot_sub;
  MirTaskRequestRetryPub::SharedPtr move_robot_retry;

  void move_robot(MiRTaskRequest::UniquePtr msg);

  // Publisher facing the fleet manager, which will output appropriate PoseStamped messages after translating from RMF Waypoint representations
  using FleetManagerPub = rclcpp::Publisher<geometry_msgs::msg::PoseStamped>;

  FleetManagerPub::SharedPtr fleet_manager_publisher; 

  // Clarify the relevance of this function (TODO)
  void test_task_receive_response(
      const SubmitTrajectoryClient::SharedFuture& response);

};

} // namespace mir_rmf_fleet_adapter

#endif // SRC__MIR_RMF_FLEET_ADAPTER__FLEETADAPTERNODE_HPP