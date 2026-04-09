/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  Copyright (c) 2021, Carlos Alvarez, Juan Galvis.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <explore/explore.h>

#include <limits>
#include <thread>

inline static bool same_point(const geometry_msgs::msg::Point& one,
                              const geometry_msgs::msg::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.01;
}

namespace explore
{
Explore::Explore()
  : Node("explore_node")
  , logger_(this->get_logger())
  , tf_buffer_(this->get_clock())
  , tf_listener_(tf_buffer_)
  , costmap_client_(*this, &tf_buffer_)
  , prev_distance_(0)
  , last_markers_count_(0)
  , active_return_to_init_(false)
  , active_exploration_(false)
  , returning_to_initial_pose_(false)
  , visited_frontier_count_(0)
{
  double timeout;
  double min_frontier_size;
  this->declare_parameter<float>("planner_frequency", 1.0);
  this->declare_parameter<float>("progress_timeout", 30.0);
  this->declare_parameter<bool>("visualize", false);
  this->declare_parameter<float>("potential_scale", 1e-3);
  this->declare_parameter<float>("orientation_scale", 0.0);
  this->declare_parameter<float>("gain_scale", 1.0);
  this->declare_parameter<float>("min_frontier_size", 0.5);
  this->declare_parameter<bool>("return_to_init", false);

  this->get_parameter("planner_frequency", planner_frequency_);
  this->get_parameter("progress_timeout", timeout);
  this->get_parameter("visualize", visualize_);
  this->get_parameter("potential_scale", potential_scale_);
  this->get_parameter("orientation_scale", orientation_scale_);
  this->get_parameter("gain_scale", gain_scale_);
  this->get_parameter("min_frontier_size", min_frontier_size);
  this->get_parameter("return_to_init", return_to_init_);
  this->get_parameter("robot_base_frame", robot_base_frame_);

  progress_timeout_ = timeout;
  move_base_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
          this, ACTION_NAME);

  search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),
                                                 potential_scale_, gain_scale_,
                                                 min_frontier_size, logger_);

  if (visualize_) {
    marker_array_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("explore/"
                                                                     "frontier"
                                                                     "s",
                                                                     10);
  }

  RCLCPP_INFO(logger_, "Waiting to connect to move_base nav2 server");
  move_base_client_->wait_for_action_server();
  RCLCPP_INFO(logger_, "Connected to move_base nav2 server");

  exploring_timer_ = this->create_wall_timer(
      std::chrono::milliseconds((uint16_t)(1000.0 / planner_frequency_)),
      [this]() { makePlan(); });
  exploring_timer_->cancel();

  explore_action_server_ = rclcpp_action::create_server<ExploreAction>(
      this, "explore",
      std::bind(&Explore::handleGoal, this, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&Explore::handleCancel, this, std::placeholders::_1),
      std::bind(&Explore::handleAccepted, this, std::placeholders::_1));
}

Explore::~Explore()
{
  move_base_client_->async_cancel_all_goals();
  exploring_timer_->cancel();
}

void Explore::visualizeFrontiers(
    const std::vector<frontier_exploration::Frontier>& frontiers)
{
  const auto blue = std_msgs::msg::ColorRGBA().set__b(1.0).set__a(0.5);
  const auto red = std_msgs::msg::ColorRGBA().set__r(1.0).set__a(0.5);
  const auto green = std_msgs::msg::ColorRGBA().set__g(1.0).set__a(0.5);

  RCLCPP_DEBUG(logger_, "visualising %lu frontiers", frontiers.size());
  visualization_msgs::msg::MarkerArray markers_msg;
  std::vector<visualization_msgs::msg::Marker>& markers = markers_msg.markers;
  visualization_msgs::msg::Marker m;

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = this->now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // m.lifetime defaults to 0, means lives forever
  m.frame_locked = true;

  // weighted frontiers are always sorted
  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

  m.action = visualization_msgs::msg::Marker::ADD;
  size_t id = 0;
  for (auto& frontier : frontiers) {
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.id = int(id);
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points = frontier.points;
    if (goalOnBlacklist(frontier.centroid)) {
      m.color = red;
    } else {
      m.color = blue;
    }
    markers.push_back(m);
    ++id;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.id = int(id);
    m.pose.position = frontier.centroid;
    // scale frontier according to its cost (costier frontiers will be smaller)
    double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.points = {};
    m.color = green;
    markers.push_back(m);
    ++id;
  }
  size_t current_markers_count = markers.size();

  // delete previous markers, which are now unused
  m.action = visualization_msgs::msg::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_->publish(markers_msg);
}

void Explore::makePlan()
{
  if (!active_exploration_ || !active_goal_handle_ ||
      active_goal_handle_->is_canceling() || returning_to_initial_pose_) {
    return;
  }

  // find frontiers
  auto pose = costmap_client_.getRobotPose();
  // get frontiers sorted according to cost
  auto frontiers = search_.searchFrom(pose.position);
  RCLCPP_DEBUG(logger_, "found %lu frontiers", frontiers.size());
  for (size_t i = 0; i < frontiers.size(); ++i) {
    RCLCPP_DEBUG(logger_, "frontier %zd cost: %f", i, frontiers[i].cost);
  }

  if (frontiers.empty()) {
    RCLCPP_WARN(logger_, "No frontiers found, stopping.");
    completeExploration("No frontiers remaining");
    return;
  }

  // publish frontiers as visualization markers
  if (visualize_) {
    visualizeFrontiers(frontiers);
  }

  // find non blacklisted frontier
  auto frontier =
      std::find_if_not(frontiers.begin(), frontiers.end(),
                       [this](const frontier_exploration::Frontier& f) {
                         return goalOnBlacklist(f.centroid);
                       });
  if (frontier == frontiers.end()) {
    RCLCPP_WARN(logger_, "All frontiers traversed/tried out, stopping.");
    completeExploration("All frontiers exhausted");
    return;
  }
  geometry_msgs::msg::Point target_position = frontier->centroid;

  // time out if we are not making any progress
  bool same_goal = same_point(prev_goal_, target_position);

  prev_goal_ = target_position;
  if (!same_goal || prev_distance_ > frontier->min_distance) {
    // we have different goal or we made some progress
    last_progress_ = this->now();
    prev_distance_ = frontier->min_distance;
  }
  // black list if we've made no progress for a long time
  if (this->now() - last_progress_ > tf2::durationFromSec(progress_timeout_)) {
    frontier_blacklist_.push_back(target_position);
    RCLCPP_DEBUG(logger_, "Adding current goal to black list");
    makePlan();
    return;
  }

  // we don't need to do anything if we still pursuing the same goal
  if (same_goal) {
    return;
  }

  RCLCPP_DEBUG(logger_, "Sending goal to move base nav2");

  // send goal to move_base if we have something new to pursue
  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = target_position;
  goal.pose.pose.orientation.w = 1.;
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  // send_goal_options.goal_response_callback =
  // std::bind(&Explore::goal_response_callback, this, _1);
  // send_goal_options.feedback_callback =
  //   std::bind(&Explore::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
      [this,
       target_position](const NavigationGoalHandle::WrappedResult& result) {
        reachedGoal(result, target_position);
      };
  move_base_client_->async_send_goal(goal, send_goal_options);
  publishFeedback("navigating_to_frontier", &target_position, frontiers.size());
}

void Explore::returnToInitialPose()
{
  RCLCPP_INFO(logger_, "Returning to initial pose.");
  publishFeedback("returning_to_origin");

  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = initial_pose_.position;
  goal.pose.pose.orientation = initial_pose_.orientation;
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback =
      [this](const NavigationGoalHandle::WrappedResult& result) {
        reachedInitialPose(result);
      };
  move_base_client_->async_send_goal(goal, send_goal_options);
}
bool Explore::goalOnBlacklist(const geometry_msgs::msg::Point& goal)
{
  constexpr static size_t tolerace = 5;
  nav2_costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

  // check if a goal is on the blacklist for goals that we're pursuing
  for (auto& frontier_goal : frontier_blacklist_) {
    double x_diff = fabs(goal.x - frontier_goal.x);
    double y_diff = fabs(goal.y - frontier_goal.y);

    if (x_diff < tolerace * costmap2d->getResolution() &&
        y_diff < tolerace * costmap2d->getResolution())
      return true;
  }
  return false;
}

void Explore::reachedGoal(const NavigationGoalHandle::WrappedResult& result,
                          const geometry_msgs::msg::Point& frontier_goal)
{
  if (!active_exploration_ || !active_goal_handle_) {
    return;
  }

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_DEBUG(logger_, "Goal was successful");
      ++visited_frontier_count_;
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_DEBUG(logger_, "Goal was aborted");
      frontier_blacklist_.push_back(frontier_goal);
      RCLCPP_DEBUG(logger_, "Adding current goal to black list");
      makePlan();
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_DEBUG(logger_, "Goal was canceled");
      if (active_goal_handle_->is_canceling()) {
        cancelExploration("Exploration goal canceled");
      }
      return;
    default:
      RCLCPP_WARN(logger_, "Unknown result code from move base nav2");
      abortExploration("unknown_navigation_result",
                       "Unknown result code from NavigateToPose");
      return;
  }
  makePlan();
}

void Explore::reachedInitialPose(const NavigationGoalHandle::WrappedResult& result)
{
  if (!active_goal_handle_) {
    return;
  }

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(logger_, "Successfully returned to initial pose.");
      publishFeedback("returned_to_origin");
      active_exploration_ = false;
      returning_to_initial_pose_ = false;
      active_return_to_init_ = false;
      {
        auto action_result = std::make_shared<ExploreAction::Result>();
        action_result->success = true;
        action_result->status = "completed";
        action_result->message =
            "Exploration complete and robot returned to origin";
        action_result->frontier_count_visited =
            static_cast<uint32_t>(visited_frontier_count_);
        active_goal_handle_->succeed(action_result);
        active_goal_handle_.reset();
      }
      return;
    case rclcpp_action::ResultCode::CANCELED:
      cancelExploration("Return-to-origin goal canceled");
      return;
    case rclcpp_action::ResultCode::ABORTED:
      abortExploration("return_to_origin_failed",
                       "Failed to return to initial pose");
      return;
    default:
      abortExploration("return_to_origin_failed",
                       "Unknown result while returning to initial pose");
      return;
  }
}

bool Explore::captureInitialPose()
{
  geometry_msgs::msg::TransformStamped transform_stamped;
  std::string map_frame = costmap_client_.getGlobalFrameID();
  try {
    transform_stamped = tf_buffer_.lookupTransform(
        map_frame, robot_base_frame_, tf2::TimePointZero);
    initial_pose_.position.x = transform_stamped.transform.translation.x;
    initial_pose_.position.y = transform_stamped.transform.translation.y;
    initial_pose_.orientation = transform_stamped.transform.rotation;
    return true;
  } catch (tf2::TransformException& ex) {
    RCLCPP_ERROR(logger_, "Couldn't find transform from %s to %s: %s",
                 map_frame.c_str(), robot_base_frame_.c_str(), ex.what());
    return false;
  }
}

void Explore::publishFeedback(const std::string& state,
                              const geometry_msgs::msg::Point* target_position,
                              size_t frontier_count_discovered)
{
  if (!active_goal_handle_ || !active_exploration_) {
    return;
  }

  auto feedback = std::make_shared<ExploreAction::Feedback>();
  feedback->state = state;
  feedback->frontier_count_discovered =
      static_cast<uint32_t>(frontier_count_discovered);
  feedback->frontier_count_blacklisted =
      static_cast<uint32_t>(frontier_blacklist_.size());
  feedback->current_target.header.frame_id = costmap_client_.getGlobalFrameID();
  feedback->current_target.header.stamp = this->now();
  if (target_position != nullptr) {
    feedback->current_target.pose.position = *target_position;
    feedback->current_target.pose.orientation.w = 1.0;
  }
  active_goal_handle_->publish_feedback(feedback);
}

void Explore::startExploration(
    const std::shared_ptr<ExploreGoalHandle> goal_handle)
{
  active_goal_handle_ = goal_handle;
  active_exploration_ = true;
  returning_to_initial_pose_ = false;
  frontier_blacklist_.clear();
  prev_goal_ = geometry_msgs::msg::Point();
  prev_distance_ = std::numeric_limits<double>::infinity();
  last_progress_ = this->now();
  visited_frontier_count_ = 0;

  const auto goal = goal_handle->get_goal();
  active_return_to_init_ = goal->return_to_init;

  if (active_return_to_init_ && !captureInitialPose()) {
    abortExploration("initial_pose_unavailable",
                     "Failed to capture initial pose for return-to-origin");
    return;
  }

  RCLCPP_INFO(logger_, "Exploration action started.");
  exploring_timer_->reset();
  publishFeedback("started");
  makePlan();
}

void Explore::cancelExploration(const std::string& message)
{
  if (!active_goal_handle_) {
    return;
  }

  move_base_client_->async_cancel_all_goals();
  exploring_timer_->cancel();
  active_exploration_ = false;
  returning_to_initial_pose_ = false;

  if (active_goal_handle_->is_canceling()) {
    auto result = std::make_shared<ExploreAction::Result>();
    result->success = false;
    result->status = "canceled";
    result->message = message;
    result->frontier_count_visited = static_cast<uint32_t>(visited_frontier_count_);
    active_goal_handle_->canceled(result);
  }

  active_goal_handle_.reset();
}

void Explore::abortExploration(const std::string& status,
                               const std::string& message)
{
  if (!active_goal_handle_) {
    return;
  }

  RCLCPP_ERROR(logger_, "Exploration aborted: %s", message.c_str());
  move_base_client_->async_cancel_all_goals();
  exploring_timer_->cancel();
  active_exploration_ = false;
  returning_to_initial_pose_ = false;

  auto result = std::make_shared<ExploreAction::Result>();
  result->success = false;
  result->status = status;
  result->message = message;
  result->frontier_count_visited = static_cast<uint32_t>(visited_frontier_count_);
  active_goal_handle_->abort(result);
  active_goal_handle_.reset();
}

void Explore::completeExploration(const std::string& message)
{
  if (!active_goal_handle_) {
    return;
  }

  exploring_timer_->cancel();
  if (active_return_to_init_) {
    returning_to_initial_pose_ = true;
    returnToInitialPose();
    return;
  }

  active_exploration_ = false;
  auto result = std::make_shared<ExploreAction::Result>();
  result->success = true;
  result->status = "completed";
  result->message = message;
  result->frontier_count_visited = static_cast<uint32_t>(visited_frontier_count_);
  active_goal_handle_->succeed(result);
  active_goal_handle_.reset();
}

rclcpp_action::GoalResponse Explore::handleGoal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const ExploreAction::Goal>)
{
  if (active_exploration_ || active_goal_handle_) {
    RCLCPP_WARN(logger_, "Rejecting exploration goal because one is already active.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Explore::handleCancel(
    const std::shared_ptr<ExploreGoalHandle> goal_handle)
{
  if (active_goal_handle_ && goal_handle == active_goal_handle_) {
    RCLCPP_INFO(logger_, "Received request to cancel exploration.");
    move_base_client_->async_cancel_all_goals();
    exploring_timer_->cancel();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  return rclcpp_action::CancelResponse::REJECT;
}

void Explore::handleAccepted(
    const std::shared_ptr<ExploreGoalHandle> goal_handle)
{
  startExploration(goal_handle);
}

}  // namespace explore

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // ROS1 code
  /*
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  } */
  rclcpp::spin(
      std::make_shared<explore::Explore>());  // std::move(std::make_unique)?
  rclcpp::shutdown();
  return 0;
}
