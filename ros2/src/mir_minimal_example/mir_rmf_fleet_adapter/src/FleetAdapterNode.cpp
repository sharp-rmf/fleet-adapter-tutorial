#include <mir_rmf_fleet_adapter/FleetAdapterNode.hpp>
#include <mir_rmf_fleet_adapter/ParseGraph.hpp>

#include <rmf_traffic_ros2/StandardNames.hpp>
#include <rmf_traffic_ros2/Trajectory.hpp>
#include <rmf_traffic/agv/Planner.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <unistd.h>

namespace mir_rmf_fleet_adapter
{
//==============================================================================
// Refer to the header for more details on the mechanism of the following implementation.
std::shared_ptr<FleetAdapterNode> FleetAdapterNode::make(
    std::string fleet_name,
    const std::string &graph_file,
    rmf_traffic::agv::VehicleTraits vehicle_traits,
    rmf_traffic::Duration wait_time)
{
  const auto start_time = std::chrono::steady_clock::now();

  // Running the constructor.
  std::shared_ptr<FleetAdapterNode> fleet_adapter(
      new FleetAdapterNode(std::move(fleet_name)));

  // Initializing the Mirror and Scheduling-facing services
  auto mirror_mgr_future = rmf_traffic_ros2::schedule::make_mirror(
      *fleet_adapter, rmf_traffic::schedule::Query::Spacetime());

  auto submit_trajectory = fleet_adapter->create_client<SubmitTrajectory>(
      rmf_traffic_ros2::SubmitTrajectoryServiceName);

  // Parse the map YAML representation into an RMF Graph
  rmf_traffic::agv::Graph graph;
  std::unordered_map<std::string, std::size_t> waypoint_keys;
  if (!parse_graph(graph_file, vehicle_traits, *fleet_adapter, graph, waypoint_keys))
  {
    // Consider using std::optional here with we move out of the stone age
    return nullptr;
  }

  const auto stop_time = start_time + wait_time;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < stop_time)
  {
    // NOTE(MXG): We need to spin the node in order to get the mirror manager
    // fully initialized.
    rclcpp::spin_some(fleet_adapter);

    using namespace std::chrono_literals;
    bool ready = (mirror_mgr_future.wait_for(0s) == std::future_status::ready);
    ready &= submit_trajectory->service_is_ready();

    if (ready)
    {

      fleet_adapter->start(
          Data{
              std::move(waypoint_keys),
              mirror_mgr_future.get(),
              std::move(submit_trajectory),
              rmf_traffic::agv::Planner::Configuration{
                  graph,
                  std::move(vehicle_traits)},
              std::move(graph)});

      return fleet_adapter;
    }
  }

  RCLCPP_ERROR(
      fleet_adapter->get_logger(),
      "Mirror was not initialized in enough time [" + std::to_string(rmf_traffic::time::to_seconds(wait_time)) + "s]!");
  return nullptr;
}

//==============================================================================
FleetAdapterNode::FleetAdapterNode(std::string _fleet_name)
    : Node(_fleet_name + "_fleet_adapter"),
      fleet_name(std::move(_fleet_name))
{
  // NOTE(MXG): We need to initialize an empty node so we can spin up the
  // MirrorManagerFuture into a full MirrorManager. But also we don't want this
  // node to do anything until all its data fields are finalized, so this
  // constructor is more like a formality than a real constructor.
}

void FleetAdapterNode::start(Data _data)
{
  // We create a Data instance ( encapsulating information after full initialization )
  // We then trigger a local cache update
  data = std::make_unique<Data>(std::move(_data));
  data->mirror.update();

  // Create subscribers and Publishers to interface with external system
  move_fleet_sub = create_subscription<MiRTaskRequest>(
      "mir_fleet_adapter/task_request",
      rclcpp::SystemDefaultsQoS(),
      [&](MiRTaskRequest::UniquePtr msg) {
        this->handle_task_request(std::move(msg));
      });

  move_fleet_retry = create_publisher<MiRTaskRequest>(
      "mir_fleet_adapter/task_request");

  // Additional, optional subscriber specific to this implementation that allows us to determine the robots current pose
  pose_sub = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mir_fleet_manager/robot_pose",
      rclcpp::SystemDefaultsQoS(),
      [&](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        data->pose = *msg;
        data->pose.pose.position.x += 10; // Transformation to align the vendor map with rmf map
        data->pose.pose.position.y += 10;
        data->pose.pose.position.y = -data->pose.pose.position.y;
      });
  
  // Publisher for the fleet manager
  fleet_manager_publisher = create_publisher<geometry_msgs::msg::PoseStamped>(
    "mir_fleet_manager/waypoint_goal", 10
  );
}

// THE FOLLOWING FUNCTIONS DESCRIBE THE FLEET ADAPTER PROCESS FLOW //
// First, an external system publishes a TaskRequest to a relevant subscriber

// Then, it will submit the trajectory asynchronously to the scheduler using a SubmitTrajectoryClient, passing handle_scheduler_response as a callback function
void FleetAdapterNode::handle_task_request(MiRTaskRequest::UniquePtr msg)
{
  // handle_task_request will process this request and generate an RMF trajectory.
  RCLCPP_INFO(get_logger(), "Received Waypoint " + msg->data);

  // // In this case, the std_msg is the waypoint label as specified in traffic-editor
  std::string goal_waypoint = msg->data;

  // // We find the index of this waypoint
  const auto goal_it = data->waypoint_keys.find(goal_waypoint);

  // // If it doesn't exist, do nothing
  if (goal_it == data->waypoint_keys.end())
  {
    RCLCPP_ERROR(get_logger(), "Are you high? The goal " + goal_waypoint + " doesn't exist!");
    return;
  }

  // If it exists, then we determine the current location of the robot
  RCLCPP_INFO(get_logger(), "Finding Current Position.");

  auto start_it = data->waypoint_keys.begin();
  double min_distance = std::numeric_limits<double>::infinity();
  std::string min_waypoint = start_it->first;
  std::size_t min_waypoint_idx = start_it->second;

  for (; start_it != data->waypoint_keys.end(); ++start_it)
  {
    std::string waypoint_name = start_it->first;
    std::size_t idx = start_it->second;
    rmf_traffic::agv::Graph::Waypoint waypoint = data->graph.get_waypoint(idx);
    Eigen::Vector2d position = waypoint.get_location();
    RCLCPP_INFO(get_logger(), "Evaluating " + waypoint_name);

    // Compute the manhattan distance from robot position to waypoint
    double distance = std::abs(position(0) - data->pose.pose.position.x) +
                      std::abs(position(1) - data->pose.pose.position.y);
    if (distance < min_distance)
    {
      min_distance = distance;
      min_waypoint = waypoint_name;
      min_waypoint_idx = idx;
    }
  }
  RCLCPP_INFO(get_logger(), "Estimating fleet to be at waypoint " + min_waypoint);

  // Then, we use the agv::Planner tool to get a feasible trajectory
  using namespace std::chrono_literals;
  const auto target_time = std::chrono::steady_clock::now() + 2s;
  rmf_traffic::agv::Plan::Start start{
      target_time,
      min_waypoint_idx,
      data->pose.pose.orientation.w};
  rmf_traffic::agv::Plan::Goal goal{goal_it->second};

  RCLCPP_INFO(get_logger(), "Attempting to compute a feasible plan.");
  const auto plan = data->planner.plan(start, goal);
  if (!plan)
  {
    RCLCPP_WARN(get_logger(), "Failed to find a solution! We will retry!");
    move_fleet_retry->publish(*msg);
    return;
  }

  // If all is successful, get the results of the planning
  const std::vector<rmf_traffic::agv::Plan::Waypoint> waypoints =
      plan.get_waypoints();
  const std::vector<rmf_traffic::Trajectory> solution = plan.get_trajectories();

  // Handle edge cases
  if (solution.empty() || solution.back().duration() == rmf_traffic::Duration(0))
  {
    RCLCPP_INFO(get_logger(), "No trajectory needed!");
    return;
  }

  // Everything is peaches! Prepare to submit to Scheduler
  const auto new_target_time = std::chrono::steady_clock::now() + 2s;
  const auto time_adjustment = new_target_time - target_time;
  SubmitTrajectory::Request request_msg;
  for (auto t : solution)
  {
    t.begin()->adjust_finish_times(time_adjustment);
    request_msg.trajectories.emplace_back(rmf_traffic_ros2::convert(t));
  }

  std::string notice =
      "Generated trajectories [" + std::to_string(solution.size()) + "]";
  for (const auto &t : solution)
    notice += " | " + std::to_string(rmf_traffic::time::to_seconds(t.duration()));

  RCLCPP_INFO(get_logger(), notice);

  RCLCPP_INFO(get_logger(), "Submitting Plan to Scheduler");
  data->submit_trajectory->async_send_request(
      std::make_shared<SubmitTrajectory::Request>(std::move(request_msg)),
      [&](const SubmitTrajectoryClient::SharedFuture response) {
        this->handle_scheduler_response(response);
      });

  // Lets assume the scheduler accepts for now!
  // if ( scheduler accepts ) {
  RCLCPP_INFO(get_logger(), "Moving Fleet.");
  handle_move_fleet(waypoints);
  // } else {}
}

void FleetAdapterNode::handle_scheduler_response(const SubmitTrajectoryClient::SharedFuture &response)
{
  const auto response_msg = response.get();
  if (response_msg->accepted)
  {
    RCLCPP_INFO(get_logger(), "Response: accepted");
  }
  else
  {
    std::string error_msg = "Response: " + response_msg->error + ". Conflicts:";
    for (const auto &conflict : response_msg->conflicts)
    {
      error_msg += "\n -- " + std::to_string(conflict.index) + " @ " + std::to_string(conflict.time);
    }
    RCLCPP_INFO(get_logger(), error_msg);
  }
}

void FleetAdapterNode::handle_move_fleet(std::vector<rmf_traffic::agv::Plan::Waypoint> waypoints)
{
  for (const auto &wp : waypoints)
  {
    geometry_msgs::msg::PoseStamped goal;
    const auto p = wp.position();
    goal.pose.position.x = p[0]; 
    goal.pose.position.y = p[1] + 20;
    goal.pose.orientation.w = p[2];
    goal.header.stamp = this->now(); // This is wrong, need to find out how to convert rmf_traffic::Time to ROS2 message
    goal.header.frame_id = "map";
    fleet_manager_publisher->publish(goal);
    usleep(800000);
    
  }
}

// //==============================================================================
} // namespace mir_rmf_fleet_adapter