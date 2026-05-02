#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/exceptions.hpp"
#include "tf2/utils.h"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_ros/transform_listener.hpp"

#include "rt2_navigation_interfaces/action/navigate_to_pose.hpp"

using namespace std::chrono_literals;

namespace rt2_navigation_assignment
{

class NavigationActionServer : public rclcpp::Node
{
public:
  using NavigateToPose = rt2_navigation_interfaces::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;

  explicit NavigationActionServer(const rclcpp::NodeOptions & options)
  : Node("navigation_action_server", options)
  {
    read_parameters();

    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions odom_options;
    odom_options.callback_group = callback_group_;

    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&NavigationActionServer::odom_callback, this, std::placeholders::_1),
      odom_options);

    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    action_server_ = rclcpp_action::create_server<NavigateToPose>(
      this,
      action_name_,
      std::bind(&NavigationActionServer::goal_callback, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&NavigationActionServer::cancel_callback, this, std::placeholders::_1),
      std::bind(&NavigationActionServer::accepted_callback, this, std::placeholders::_1),
      rcl_action_server_get_default_options(),
      callback_group_);

    RCLCPP_INFO(this->get_logger(), "Navigation action server ready on '%s'", action_name_.c_str());
    RCLCPP_INFO(
      this->get_logger(), "Using odometry topic %s and velocity topic %s",
      odom_topic_.c_str(), cmd_vel_topic_.c_str());
  }

private:
  enum class MotionStep
  {
    FACE_TARGET,
    GO_TO_TARGET,
    MATCH_FINAL_YAW
  };

  struct Pose2D
  {
    double x{0.0};
    double y{0.0};
    double theta{0.0};
  };

  struct GoalError
  {
    double distance{0.0};
    double heading{0.0};
    double yaw{0.0};
  };

  void read_parameters()
  {
    action_name_ = this->declare_parameter<std::string>("action_name", "navigate_to_pose");
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/odom");
    cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");

    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_footprint");
    goal_frame_ = this->declare_parameter<std::string>("target_frame", "navigation_goal");

    publish_odom_tf_ = this->declare_parameter<bool>("publish_odom_tf", true);
    loop_hz_ = this->declare_parameter<double>("control_frequency", 20.0);

    k_linear_ = this->declare_parameter<double>("linear_gain", 0.26);
    k_heading_ = this->declare_parameter<double>("angular_gain", 0.32);
    k_yaw_ = this->declare_parameter<double>("yaw_gain", 0.22);

    max_v_ = this->declare_parameter<double>("max_linear_speed", 0.16);
    max_w_ = this->declare_parameter<double>("max_angular_speed", 0.14);

    xy_ok_ = this->declare_parameter<double>("xy_tolerance", 0.20);
    yaw_ok_ = this->declare_parameter<double>("yaw_tolerance", 0.30);
    turn_again_limit_ = this->declare_parameter<double>("rotate_in_place_threshold", 0.50);
    heading_ok_ = this->declare_parameter<double>("heading_tolerance", 0.12);
  }

  static double normalize(double angle)
  {
    constexpr double pi = 3.14159265358979323846;
    while (angle > pi) {
      angle -= 2.0 * pi;
    }
    while (angle < -pi) {
      angle += 2.0 * pi;
    }
    return angle;
  }

  static double limit(double value, double low, double high)
  {
    return std::max(low, std::min(value, high));
  }

  static double stamp_seconds(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
  }

  static bool valid_goal(const NavigateToPose::Goal & goal)
  {
    return std::isfinite(goal.x) && std::isfinite(goal.y) && std::isfinite(goal.theta);
  }

  static Pose2D odom_to_pose(const nav_msgs::msg::Odometry & odom)
  {
    Pose2D pose;
    pose.x = odom.pose.pose.position.x;
    pose.y = odom.pose.pose.position.y;
    const auto & q = odom.pose.pose.orientation;
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);

    pose.theta = std::atan2(siny_cosp, cosy_cosp);
    return pose;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      latest_odom_ = *msg;
      odom_ready_ = true;

      if (!msg->header.frame_id.empty()) {
        odom_frame_ = msg->header.frame_id;
      }
      if (!msg->child_frame_id.empty()) {
        base_frame_ = msg->child_frame_id;
      }
    }

    // Dynamic broadcaster: odom is used to publish the moving robot frame.
    if (publish_odom_tf_) {
      tf_broadcaster_->sendTransform(make_odom_tf(*msg));
    }
  }

  geometry_msgs::msg::TransformStamped make_odom_tf(const nav_msgs::msg::Odometry & odom)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = odom.header.stamp;
    if (stamp_seconds(transform.header.stamp) <= 0.0) {
      transform.header.stamp = this->now();
    }

    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = odom.pose.pose.position.x;
    transform.transform.translation.y = odom.pose.pose.position.y;
    transform.transform.translation.z = odom.pose.pose.position.z;
    transform.transform.rotation = odom.pose.pose.orientation;
    return transform;
  }

  geometry_msgs::msg::TransformStamped make_goal_tf(const NavigateToPose::Goal & goal)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = goal_frame_;
    transform.transform.translation.x = goal.x;
    transform.transform.translation.y = goal.y;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, goal.theta);
    q.normalize();
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    return transform;
  }

  bool latest_odom(nav_msgs::msg::Odometry & odom)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (!odom_ready_) {
      return false;
    }
    odom = latest_odom_;
    return true;
  }

  bool accept_new_goal()
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (goal_active_) {
      return false;
    }
    goal_active_ = true;
    return true;
  }

  void release_goal()
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    goal_active_ = false;
  }

  void stop_robot()
  {
    cmd_vel_publisher_->publish(geometry_msgs::msg::Twist());
  }

  rclcpp_action::GoalResponse goal_callback(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const NavigateToPose::Goal> goal)
  {
    if (!valid_goal(*goal)) {
      RCLCPP_WARN(this->get_logger(), "Rejected goal with non-finite values");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (!accept_new_goal()) {
      RCLCPP_WARN(this->get_logger(), "Rejected goal: another target is still active");
      return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(
      this->get_logger(), "Received goal: x=%.3f y=%.3f theta=%.3f",
      goal->x, goal->y, goal->theta);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse cancel_callback(
    const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_WARN(this->get_logger(), "Cancel request received");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void accepted_callback(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    // As discussed in the components lesson, the keyboard/client and the action execution
    // must not block the container callbacks.
    std::thread(&NavigationActionServer::execute, this, goal_handle).detach();
  }

  bool wait_for_odom(
    const std::shared_ptr<GoalHandleNavigateToPose> goal_handle,
    const std::shared_ptr<NavigateToPose::Result> result)
  {
    rclcpp::Rate rate(10.0);
    const auto start = std::chrono::steady_clock::now();

    while (rclcpp::ok()) {
      nav_msgs::msg::Odometry ignored;
      if (latest_odom(ignored)) {
        return true;
      }

      if (goal_handle->is_canceling()) {
        stop_robot();
        result->success = false;
        result->message = "Goal cancelled before odometry was available";
        goal_handle->canceled(result);
        release_goal();
        return false;
      }

      if (std::chrono::steady_clock::now() - start > std::chrono::seconds(8)) {
        stop_robot();
        result->success = false;
        result->message = "No odometry received on " + odom_topic_;
        goal_handle->abort(result);
        release_goal();
        return false;
      }

      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Waiting for odometry on %s...", odom_topic_.c_str());
      rate.sleep();
    }

    return false;
  }

  bool get_goal_in_robot_frame(geometry_msgs::msg::TransformStamped & transform)
  {
    try {
      // tf2::TimePointZero follows the frames lesson: use the latest available transform.
      transform = tf_buffer_->lookupTransform(base_frame_, goal_frame_, tf2::TimePointZero, 50ms);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Could not transform %s to %s: %s",
        goal_frame_.c_str(), base_frame_.c_str(), ex.what());
      return false;
    }
  }

  GoalError compute_error(
    const geometry_msgs::msg::TransformStamped & goal_in_robot,
    const Pose2D & pose,
    const NavigateToPose::Goal & goal)
  {
    const double dx = goal_in_robot.transform.translation.x;
    const double dy = goal_in_robot.transform.translation.y;

    GoalError error;
    error.distance = std::hypot(dx, dy);
    error.heading = normalize(std::atan2(dy, dx));
    error.yaw = normalize(goal.theta - pose.theta);
    return error;
  }

  geometry_msgs::msg::Twist control_command(MotionStep & step, const GoalError & error)
  {
    geometry_msgs::msg::Twist cmd;

    if (step != MotionStep::MATCH_FINAL_YAW && error.distance <= xy_ok_) {
      step = MotionStep::MATCH_FINAL_YAW;
      stop_robot();
      RCLCPP_INFO(this->get_logger(), "Target x,y reached. Aligning final orientation...");
    }

    if (step == MotionStep::FACE_TARGET) {
      if (std::fabs(error.heading) > heading_ok_) {
        cmd.angular.z = limit(k_heading_ * error.heading, -max_w_, max_w_);
      } else {
        step = MotionStep::GO_TO_TARGET;
        RCLCPP_INFO(this->get_logger(), "Robot is facing the target. Driving to x,y...");
      }
      return cmd;
    }

    if (step == MotionStep::GO_TO_TARGET) {
      if (std::fabs(error.heading) > turn_again_limit_) {
        cmd.angular.z = limit(k_heading_ * error.heading, -max_w_, max_w_);
        step = MotionStep::FACE_TARGET;
        RCLCPP_INFO(this->get_logger(), "Heading error too large. Turning to target again...");
      } else {
        cmd.linear.x = limit(k_linear_ * error.distance, 0.0, max_v_);
        cmd.angular.z = limit(k_heading_ * error.heading, -max_w_, max_w_);
      }
      return cmd;
    }

    cmd.angular.z = limit(k_yaw_ * error.yaw, -max_w_, max_w_);
    return cmd;
  }

  void publish_feedback(
    const std::shared_ptr<GoalHandleNavigateToPose> goal_handle,
    const std::shared_ptr<NavigateToPose::Feedback> feedback,
    const Pose2D & pose,
    const GoalError & error)
  {
    feedback->current_x = pose.x;
    feedback->current_y = pose.y;
    feedback->current_theta = pose.theta;
    feedback->distance_to_goal = error.distance;
    feedback->heading_error = error.heading;
    feedback->yaw_error = error.yaw;
    goal_handle->publish_feedback(feedback);
  }

  void fill_result_pose(const std::shared_ptr<NavigateToPose::Result> result)
  {
    nav_msgs::msg::Odometry odom;
    if (latest_odom(odom)) {
      const auto pose = odom_to_pose(odom);
      result->final_x = pose.x;
      result->final_y = pose.y;
      result->final_theta = pose.theta;
    }
  }

  bool cancel_if_needed(
    const std::shared_ptr<GoalHandleNavigateToPose> goal_handle,
    const std::shared_ptr<NavigateToPose::Result> result)
  {
    if (!goal_handle->is_canceling()) {
      return false;
    }

    stop_robot();
    fill_result_pose(result);
    result->success = false;
    result->message = "Navigation goal cancelled";
    goal_handle->canceled(result);
    release_goal();
    return true;
  }

  void execute(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<NavigateToPose::Feedback>();
    auto result = std::make_shared<NavigateToPose::Result>();

    if (!wait_for_odom(goal_handle, result)) {
      return;
    }

    MotionStep step = MotionStep::FACE_TARGET;
    rclcpp::Rate rate(loop_hz_);

    while (rclcpp::ok()) {
      if (cancel_if_needed(goal_handle, result)) {
        return;
      }

      tf_broadcaster_->sendTransform(make_goal_tf(*goal));

      geometry_msgs::msg::TransformStamped goal_in_robot;
      if (!get_goal_in_robot_frame(goal_in_robot)) {
        rate.sleep();
        continue;
      }

      nav_msgs::msg::Odometry odom;
      latest_odom(odom);
      const auto pose = odom_to_pose(odom);
      const auto error = compute_error(goal_in_robot, pose, *goal);

      publish_feedback(goal_handle, feedback, pose, error);

      if (step == MotionStep::MATCH_FINAL_YAW && std::fabs(error.yaw) <= yaw_ok_) {
        stop_robot();
        fill_result_pose(result);
        result->success = true;
        result->message = "Target reached";
        goal_handle->succeed(result);
        release_goal();
        RCLCPP_INFO(this->get_logger(), "Goal reached");
        return;
      }

      cmd_vel_publisher_->publish(control_command(step, error));
      rate.sleep();
    }

    stop_robot();
    fill_result_pose(result);
    result->success = false;
    result->message = "ROS was shut down before the goal finished";
    goal_handle->abort(result);
    release_goal();
  }

  std::string action_name_;
  std::string odom_topic_;
  std::string cmd_vel_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string goal_frame_;

  bool publish_odom_tf_{true};
  double loop_hz_{20.0};
  double k_linear_{0.26};
  double k_heading_{0.32};
  double k_yaw_{0.22};
  double max_v_{0.16};
  double max_w_{0.14};
  double xy_ok_{0.20};
  double yaw_ok_{0.30};
  double turn_again_limit_{0.50};
  double heading_ok_{0.12};

  bool odom_ready_{false};
  bool goal_active_{false};
  nav_msgs::msg::Odometry latest_odom_;
  std::mutex odom_mutex_;
  std::mutex goal_mutex_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace rt2_navigation_assignment

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_navigation_assignment::NavigationActionServer)
