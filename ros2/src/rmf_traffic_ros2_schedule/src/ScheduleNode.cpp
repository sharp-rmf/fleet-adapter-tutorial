/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef SRC__RMF_TRAFFIC_SCHEDULE__SCHEDULENODE_HPP
#define SRC__RMF_TRAFFIC_SCHEDULE__SCHEDULENODE_HPP

#include <rmf_traffic/schedule/Database.hpp>

#include <rclcpp/node.hpp>

#include <rmf_traffic_msgs/msg/mirror_wakeup.hpp>

#include <rmf_traffic_msgs/srv/submit_trajectory.hpp>
#include <rmf_traffic_msgs/srv/erase_schedule.hpp>
#include <rmf_traffic_msgs/srv/mirror_update.hpp>
#include <rmf_traffic_msgs/srv/register_query.hpp>
#include <rmf_traffic_msgs/srv/mirror_update.h>
#include <rmf_traffic_msgs/srv/unregister_query.hpp>

#include <unordered_map>

namespace rmf_traffic_schedule {

//==============================================================================
class ScheduleNode : public rclcpp::Node
{
public:

  ScheduleNode();


private:

  using SubmitTrajectory = rmf_traffic_msgs::srv::SubmitTrajectory;
  using SubmitTrajectoryService = rclcpp::Service<SubmitTrajectory>;

  void submit_trajectory(
      const std::shared_ptr<rmw_request_id_t>& request_header,
      const SubmitTrajectory::Request::SharedPtr& request,
      const SubmitTrajectory::Response::SharedPtr& response);

  SubmitTrajectoryService::SharedPtr submit_trajectory_service;


  using EraseSchedule = rmf_traffic_msgs::srv::EraseSchedule;
  using EraseScheduleService = rclcpp::Service<EraseSchedule>;

  void erase_schedule(
      const std::shared_ptr<rmw_request_id_t>& request_header,
      const EraseSchedule::Request::SharedPtr& request,
      const EraseSchedule::Response::SharedPtr& response);

  EraseScheduleService::SharedPtr erase_schedule_service;


  using RegisterQuery = rmf_traffic_msgs::srv::RegisterQuery;
  using RegisterQueryService = rclcpp::Service<RegisterQuery>;

  void register_query(
      const std::shared_ptr<rmw_request_id_t>& request_header,
      const RegisterQuery::Request::SharedPtr& request,
      const RegisterQuery::Response::SharedPtr& response);

  RegisterQueryService::SharedPtr register_query_service;


  using UnregisterQuery = rmf_traffic_msgs::srv::UnregisterQuery;
  using UnregisterQueryService = rclcpp::Service<UnregisterQuery>;

  void unregister_query(
      const std::shared_ptr<rmw_request_id_t>& request_header,
      const UnregisterQuery::Request::SharedPtr& request,
      const UnregisterQuery::Response::SharedPtr& response);

  UnregisterQueryService::SharedPtr unregister_query_service;


  using MirrorUpdate = rmf_traffic_msgs::srv::MirrorUpdate;
  using MirrorUpdateService = rclcpp::Service<MirrorUpdate>;

  void mirror_update(
      const std::shared_ptr<rmw_request_id_t>& request_header,
      const MirrorUpdate::Request::SharedPtr& request,
      const MirrorUpdate::Response::SharedPtr& response);

  MirrorUpdateService::SharedPtr mirror_update_service;


  using MirrorWakeup = rmf_traffic_msgs::msg::MirrorWakeup;
  using MirrorWakeupPublisher = rclcpp::Publisher<MirrorWakeup>;
  MirrorWakeupPublisher::SharedPtr mirror_wakeup_publisher;

  void wakeup_mirrors() const;

  rmf_traffic::schedule::Database database;

  using QueryMap =
      std::unordered_map<uint64_t, rmf_traffic::schedule::Query::Spacetime>;
  // TODO(MXG): Have a way to make query registrations expire after they have
  // not been used for some set amount of time (e.g. 24 hours? 48 hours?).
  std::size_t last_query_id = 0;
  QueryMap registered_queries;

};

} // namespace rmf_traffic_schedule

#endif // SRC__RMF_TRAFFIC_SCHEDULE__SCHEDULENODE_HPP