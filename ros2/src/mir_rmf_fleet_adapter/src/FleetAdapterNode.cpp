#include "FleetAdapterNode.hpp"

#include <rmf_traffic_ros2/StandardNames.hpp>
#include <rmf_traffic_ros2/Trajectory.hpp>
#include <rmf_traffic/agv/Planner.hpp>

#include <rclcpp/rclcpp.hpp>
#include "ParseGraph.hpp"

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
              std::move(graph),
              std::move(waypoint_keys),
              std::move(vehicle_traits),
              mirror_mgr_future.get(),
              std::move(submit_trajectory)});

      return fleet_adapter;
    }
  }

  RCLCPP_ERROR(
      fleet_adapter->get_logger(),
      "Mirror was not initialized in enough time [" + std::to_string(rmf_traffic::time::to_seconds(wait_time)) + "s]!");
  return nullptr;
}

//==============================================================================
} // namespace mir_rmf_fleet_adapter