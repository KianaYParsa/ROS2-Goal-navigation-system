#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include <sys/select.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "rt2_navigation_interfaces/action/navigate_to_pose.hpp"

using namespace std::chrono_literals;

namespace rt2_navigation_assignment
{

class NavigationClientComponent : public rclcpp::Node
{
public:
  using NavigateToPose = rt2_navigation_interfaces::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit NavigationClientComponent(const rclcpp::NodeOptions & options)
  : Node("navigation_user_interface", options)
  {
    action_name_ = this->declare_parameter<std::string>("action_name", "navigate_to_pose");
    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, action_name_);

    RCLCPP_INFO(this->get_logger(), "Navigation client component ready");
    start_input_thread();
  }

  ~NavigationClientComponent() override
  {
    input_running_ = false;
    if (input_thread_.joinable()) {
      input_thread_.join();
    }
  }

private:
  enum class Command
  {
    EMPTY,
    GOAL,
    CANCEL,
    STATUS,
    HELP,
    QUIT,
    UNKNOWN
  };

  struct UserCommand
  {
    Command type{Command::EMPTY};
    std::string word;
    double x{0.0};
    double y{0.0};
    double theta{0.0};
    bool numbers_read{false};
  };

  static std::string trim(std::string text)
  {
    const auto first = text.find_first_not_of(" \t\n\r");
    if (first == std::string::npos) {
      return "";
    }
    const auto last = text.find_last_not_of(" \t\n\r");
    return text.substr(first, last - first + 1);
  }

  static std::string lower(std::string text)
  {
    std::transform(
      text.begin(), text.end(), text.begin(),
      [](unsigned char c) {return static_cast<char>(std::tolower(c));});
    return text;
  }

  static bool terminal_ready(std::chrono::milliseconds timeout)
  {
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);

    timeval tv;
    tv.tv_sec = static_cast<long>(timeout.count() / 1000);
    tv.tv_usec = static_cast<long>((timeout.count() % 1000) * 1000);

    const int ready = select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv);
    return ready > 0 && FD_ISSET(STDIN_FILENO, &fds);
  }

  static UserCommand parse_line(const std::string & line)
  {
    UserCommand parsed;
    const auto clean_line = trim(line);
    if (clean_line.empty()) {
      return parsed;
    }

    std::istringstream input(clean_line);
    input >> parsed.word;
    parsed.word = lower(parsed.word);

    if (parsed.word == "goal" || parsed.word == "goto" || parsed.word == "target") {
      parsed.type = Command::GOAL;
      parsed.numbers_read = static_cast<bool>(input >> parsed.x >> parsed.y >> parsed.theta);
    } else if (parsed.word == "cancel" || parsed.word == "stop") {
      parsed.type = Command::CANCEL;
    } else if (parsed.word == "status") {
      parsed.type = Command::STATUS;
    } else if (parsed.word == "help" || parsed.word == "?") {
      parsed.type = Command::HELP;
    } else if (parsed.word == "quit" || parsed.word == "exit") {
      parsed.type = Command::QUIT;
    } else {
      parsed.type = Command::UNKNOWN;
    }

    return parsed;
  }

  void start_input_thread()
  {
    if (!isatty(STDIN_FILENO)) {
      RCLCPP_WARN(
        this->get_logger(),
        "No interactive terminal detected. Load this component in a visible container to use the UI.");
      return;
    }

    input_running_ = true;
    input_thread_ = std::thread(&NavigationClientComponent::input_loop, this);
  }

  void show_help()
  {
    std::lock_guard<std::mutex> lock(cout_mutex_);
    std::cout << "\nResearch Track II navigation UI\n";
    std::cout << "  goal X Y THETA   send a pose in odom\n";
    std::cout << "  cancel           cancel the active goal\n";
    std::cout << "  status           print client state\n";
    std::cout << "  help             print this message\n";
    std::cout << "  quit             stop the input thread\n";
    std::cout << "Example: goal 6.0 3.0 1.57\n" << std::flush;
  }

  void input_loop()
  {
    show_help();

    while (rclcpp::ok() && input_running_) {
      {
        std::lock_guard<std::mutex> lock(cout_mutex_);
        std::cout << "\nrt2-ui> " << std::flush;
      }

      while (rclcpp::ok() && input_running_ && !terminal_ready(200ms)) {
      }

      if (!rclcpp::ok() || !input_running_) {
        break;
      }

      std::string line;
      if (!std::getline(std::cin, line)) {
        std::cin.clear();
        continue;
      }

      handle_command(parse_line(line));
    }

    input_running_ = false;
  }

  void handle_command(const UserCommand & command)
  {
    switch (command.type) {
      case Command::EMPTY:
        return;

      case Command::GOAL:
        handle_goal_input(command);
        return;

      case Command::CANCEL:
        cancel_goal();
        return;

      case Command::STATUS:
        RCLCPP_INFO(
          this->get_logger(), "Client status: active=%s cancel_sent=%s",
          goal_active_ ? "true" : "false",
          cancel_sent_ ? "true" : "false");
        return;

      case Command::HELP:
        show_help();
        return;

      case Command::QUIT:
        input_running_ = false;
        RCLCPP_INFO(this->get_logger(), "Keyboard UI stopped");
        return;

      case Command::UNKNOWN:
        RCLCPP_WARN(this->get_logger(), "Unknown command '%s'", command.word.c_str());
        return;
    }
  }

  void handle_goal_input(const UserCommand & command)
  {
    if (!command.numbers_read) {
      RCLCPP_WARN(this->get_logger(), "Usage: goal X Y THETA");
      return;
    }

    if (!std::isfinite(command.x) || !std::isfinite(command.y) || !std::isfinite(command.theta)) {
      RCLCPP_WARN(this->get_logger(), "Goal values must be finite numbers");
      return;
    }

    send_goal(command.x, command.y, command.theta);
  }

  bool reserve_goal()
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (goal_active_) {
      RCLCPP_WARN(this->get_logger(), "A goal is already active. Cancel it before sending another.");
      return false;
    }

    goal_active_ = true;
    cancel_sent_ = false;
    goal_handle_.reset();
    return true;
  }

  void clear_goal()
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    goal_active_ = false;
    cancel_sent_ = false;
    goal_handle_.reset();
  }

  bool send_goal(double x, double y, double theta)
  {
    if (!action_client_->wait_for_action_server(3s)) {
      RCLCPP_ERROR(this->get_logger(), "Action server '%s' not available", action_name_.c_str());
      return false;
    }

    if (!reserve_goal()) {
      return false;
    }

    NavigateToPose::Goal goal;
    goal.x = x;
    goal.y = y;
    goal.theta = theta;

    using namespace std::placeholders;
    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback =
      std::bind(&NavigationClientComponent::goal_response_callback, this, _1);
    options.feedback_callback =
      std::bind(&NavigationClientComponent::feedback_callback, this, _1, _2);
    options.result_callback =
      std::bind(&NavigationClientComponent::result_callback, this, _1);

    action_client_->async_send_goal(goal, options);

    RCLCPP_INFO(this->get_logger(), "Goal sent: x=%.3f y=%.3f theta=%.3f", x, y, theta);
    return true;
  }

  bool cancel_goal()
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (!goal_active_ || cancel_sent_) {
      RCLCPP_WARN(this->get_logger(), "No active goal to cancel");
      return false;
    }

    cancel_sent_ = true;
    if (goal_handle_) {
      action_client_->async_cancel_goal(goal_handle_);
    } else {
      action_client_->async_cancel_all_goals();
    }

    RCLCPP_WARN(this->get_logger(), "Cancel request sent");
    return true;
  }

  void goal_response_callback(GoalHandleNavigateToPose::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_WARN(this->get_logger(), "Goal rejected");
      clear_goal();
      return;
    }

    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      goal_handle_ = goal_handle;
    }

    RCLCPP_INFO(this->get_logger(), "Goal accepted");
  }

  void feedback_callback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Feedback: pose=(%.2f, %.2f, %.2f), distance=%.2f, heading=%.2f, yaw_error=%.2f",
      feedback->current_x,
      feedback->current_y,
      feedback->current_theta,
      feedback->distance_to_goal,
      feedback->heading_error,
      feedback->yaw_error);
  }

  void result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
  {
    clear_goal();

    if (!result.result) {
      RCLCPP_WARN(this->get_logger(), "Action finished without result data");
      return;
    }

    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(), "Result: SUCCEEDED - %s", result.result->message.c_str());
    } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
      RCLCPP_WARN(this->get_logger(), "Result: CANCELED - %s", result.result->message.c_str());
    } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
      RCLCPP_WARN(this->get_logger(), "Result: ABORTED - %s", result.result->message.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "Result: unknown status");
    }

    RCLCPP_INFO(
      this->get_logger(), "Final pose: x=%.3f y=%.3f theta=%.3f",
      result.result->final_x,
      result.result->final_y,
      result.result->final_theta);
  }

  std::string action_name_;

  std::atomic_bool goal_active_{false};
  std::atomic_bool cancel_sent_{false};
  std::atomic_bool input_running_{false};

  std::mutex goal_mutex_;
  std::mutex cout_mutex_;
  GoalHandleNavigateToPose::SharedPtr goal_handle_;

  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  std::thread input_thread_;
};

}  // namespace rt2_navigation_assignment

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_navigation_assignment::NavigationClientComponent)
