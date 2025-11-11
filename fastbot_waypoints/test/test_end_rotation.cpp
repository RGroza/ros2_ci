#include <chrono>
#include <cmath>
#include <fastbot_msgs/action/navigate_to_pose.hpp>
#include <gtest/gtest.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;
using NavigateToPose = fastbot_msgs::action::NavigateToPose;

class EndRotationTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("end_rotation_test");
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/fastbot/odom", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          last_pose_ = *msg;
          got_odom_ = true;
        });

    client_ = rclcpp_action::create_client<NavigateToPose>(node_, "fastbot_as");
  }

  void TearDown() override {
    node_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  nav_msgs::msg::Odometry last_pose_;
  bool got_odom_{false};
};

double normalize_angle(double angle) {
  if (angle > M_PI)
    angle -= 2 * M_PI;
  else if (angle < -M_PI)
    angle += 2 * M_PI;
  return angle;
}

TEST_F(EndRotationTest, EndYawWithinThreshold) {
  ASSERT_TRUE(client_->wait_for_action_server(5s));

  auto goal = NavigateToPose::Goal();
  goal.position.x = 1.0;
  goal.position.y = 1.0;

  auto send_goal_future = client_->async_send_goal(goal);
  ASSERT_EQ(rclcpp::spin_until_future_complete(node_, send_goal_future, 5s),
            rclcpp::FutureReturnCode::SUCCESS);
  auto goal_handle = send_goal_future.get();
  ASSERT_TRUE(goal_handle);

  auto result_future = client_->async_get_result(goal_handle);
  ASSERT_EQ(rclcpp::spin_until_future_complete(node_, result_future, 60s),
            rclcpp::FutureReturnCode::SUCCESS);
  auto result = result_future.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(result.result->success);

  // Wait for odom
  auto start = node_->now();
  while (!got_odom_ && (node_->now() - start).seconds() < 3.0)
    rclcpp::spin_some(node_);

  ASSERT_TRUE(got_odom_);

  const auto &q = last_pose_.pose.pose.orientation;
  const auto yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  double desired_yaw = std::atan2(goal.position.y, goal.position.x);
  double err_yaw = normalize_angle(normalize_angle(desired_yaw - yaw) + M_PI_2);

  RCLCPP_INFO(node_->get_logger(), "Yaw error: %.3f rad", err_yaw);
  EXPECT_LT(err_yaw, 0.4); // <-- Change to 0.001 for FAIL condition
}
