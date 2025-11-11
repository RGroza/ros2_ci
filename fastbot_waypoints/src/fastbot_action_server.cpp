#include <cmath>
#include <fastbot_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using NavigateToPose = fastbot_msgs::action::NavigateToPose;
using GoalHandleNavigate = rclcpp_action::ServerGoalHandle<NavigateToPose>;

class FastbotActionServer : public rclcpp::Node {
public:
  FastbotActionServer() : Node("fastbot_action_server") {
    this->declare_parameter<double>("yaw_precision", M_PI / 90.0);
    this->declare_parameter<double>("dist_precision", 0.05);

    yaw_precision_ = this->get_parameter("yaw_precision").as_double();
    dist_precision_ = this->get_parameter("dist_precision").as_double();

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/fastbot/cmd_vel", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/fastbot/odom", 10,
        std::bind(&FastbotActionServer::odomCallback, this,
                  std::placeholders::_1));

    action_server_ = rclcpp_action::create_server<NavigateToPose>(
        this, "fastbot_as",
        std::bind(&FastbotActionServer::handleGoal, this, std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&FastbotActionServer::handleCancel, this,
                  std::placeholders::_1),
        std::bind(&FastbotActionServer::handleAccepted, this,
                  std::placeholders::_1));

    last_log_time_ = this->get_clock()->now();

    RCLCPP_INFO(this->get_logger(), "Fastbot action server started");
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
  geometry_msgs::msg::Point position_;
  double yaw_{0.0};
  double yaw_precision_;
  double dist_precision_;
  rclcpp::Time last_log_time_;

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    position_ = msg->pose.pose.position;
    const auto &q = msg->pose.pose.orientation;
    yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  rclcpp_action::GoalResponse
  handleGoal(const rclcpp_action::GoalUUID &,
             std::shared_ptr<const NavigateToPose::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal: x=%.2f, y=%.2f",
                goal->position.x, goal->position.y);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handleCancel(const std::shared_ptr<GoalHandleNavigate>) {
    RCLCPP_INFO(this->get_logger(), "Goal canceled");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandleNavigate> goal_handle) {
    std::thread{std::bind(&FastbotActionServer::execute, this, goal_handle)}
        .detach();
  }

  double normalize_angle(double angle) {
    if (angle > M_PI)
      angle -= 2 * M_PI;
    else if (angle < -M_PI)
      angle += 2 * M_PI;
    return angle;
  }

  void execute(const std::shared_ptr<GoalHandleNavigate> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();

    rclcpp::Rate rate(25);
    auto feedback = std::make_shared<NavigateToPose::Feedback>();
    auto result = std::make_shared<NavigateToPose::Result>();

    geometry_msgs::msg::Twist twist;
    bool success = true;

    while (rclcpp::ok()) {
      double desired_yaw = std::atan2(goal->position.y - position_.y,
                                      goal->position.x - position_.x);
      double err_yaw =
          normalize_angle(normalize_angle(desired_yaw - yaw_) + M_PI_2);
      double err_pos = std::hypot(goal->position.x - position_.x,
                                  goal->position.y - position_.y);

      // Logging desired and error yaw values
      // auto now = this->get_clock()->now();
      // if (now - last_log_time_ > rclcpp::Duration::from_seconds(0.5)) {
      //   RCLCPP_INFO(this->get_logger(), "Desired yaw: %.2f, Error yaw: %.2f",
      //               desired_yaw, err_yaw);
      //   last_log_time_ = now;
      // }

      if (goal_handle->is_canceling()) {
        twist.linear.x = 0.0;
        twist.angular.z = 0.0;
        cmd_pub_->publish(twist);
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      if (err_pos < dist_precision_ && std::fabs(err_yaw) < yaw_precision_) {
        twist.linear.x = 0.0;
        twist.angular.z = 0.0;
        cmd_pub_->publish(twist);
        break;
      }

      if (std::fabs(err_yaw) > yaw_precision_) {
        feedback->state = "fix yaw";
        twist.linear.x = 0.0;
        twist.angular.z = (err_yaw > 0 ? 0.5 : -0.5);
      } else {
        feedback->state = "go to point";
        twist.linear.x = 0.4;
        twist.angular.z = 0.0;
      }

      feedback->current_position = position_;
      goal_handle->publish_feedback(feedback);
      cmd_pub_->publish(twist);
      rate.sleep();
    }

    result->success = success;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FastbotActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
