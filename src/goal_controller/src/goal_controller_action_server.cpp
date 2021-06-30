// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Standard libraries.
#include <chrono>
#include <cmath>
#include <memory>

// Ros libraries.
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "ros_demo_interfaces/action/move_robot.hpp"
#include "turtlesim/msg/pose.hpp"

constexpr const char *TURTLESIM_CMD_TOPIC = "/turtle1/cmd_vel";
constexpr const char *TURTLESIM_POSE_TOPIC = "/turtle1/pose";
constexpr const double DISTANCE_THRESHOLD = 0.5f;
constexpr const double LINEAR_SPEED = 1.5f;
constexpr const double MAX_LINEAR_SPEED = 4.0f;
constexpr const double ANGULAR_SPEED = 3.0f;
constexpr const double MAX_ANGULAR_SPEED = 4.0f;

using namespace std::chrono_literals;

namespace goal_controller {
class GoalControllerServer : public rclcpp::Node {
   public:
    using MoveRobot = ros_demo_interfaces::action::MoveRobot;
    using GoalHandleMoveRobot = rclcpp_action::ServerGoalHandle<MoveRobot>;

    explicit GoalControllerServer(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("goal_controller_action_server", options) {
        using namespace std::placeholders;

        // Parameters Setup.
        this->declare_parameter<double>("distance_threshold",
                                        DISTANCE_THRESHOLD);
        this->declare_parameter<double>("linear_speed", LINEAR_SPEED);
        this->declare_parameter<double>("max_linear_speed", MAX_LINEAR_SPEED);
        this->declare_parameter<double>("angular_speed", ANGULAR_SPEED);
        this->declare_parameter<double>("max_angular_speed", MAX_ANGULAR_SPEED);

        timer_ = this->create_wall_timer(
            1000ms,
            std::bind(&GoalControllerServer::parameter_update_callback, this));

        // Topics and services.
        rclcpp::QoS qos(rclcpp::KeepLast(10));
        turtlesim_cmd_vel_pub_ =
            this->create_publisher<geometry_msgs::msg::Twist>(
                TURTLESIM_CMD_TOPIC, qos);
        turtlesim_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            TURTLESIM_POSE_TOPIC, qos,
            std::bind(&GoalControllerServer::turtlesim_pose_callback, this,
                      std::placeholders::_1));

        // Create Action server.
        this->action_server_ = rclcpp_action::create_server<MoveRobot>(
            this->get_node_base_interface(), this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(), "goal_controller",
            std::bind(&GoalControllerServer::handle_goal, this, _1, _2),
            std::bind(&GoalControllerServer::handle_cancel, this, _1),
            std::bind(&GoalControllerServer::handle_accepted, this, _1));
        RCLCPP_INFO(this->get_logger(),
                    "Starting goal_controller_action_server.");
    }

   private:
    /// @brief Parameters of the node..
    double distance_threshold_;
    double linear_speed_;
    double max_linear_speed_;
    double angular_speed_;
    double max_angular_speed_;

    rclcpp::TimerBase::SharedPtr timer_;

    /// @brief Action server for accepting new commands.
    rclcpp_action::Server<MoveRobot>::SharedPtr action_server_;
    /// @brief Publisher to turtlesim.
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
        turtlesim_cmd_vel_pub_;
    /// @brief Subscriber to turtlesim pose.
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtlesim_pose_sub_;

    /// @brief Current pose of the robot.
    turtlesim::msg::Pose turtlesim_current_pose_;

    /**
     * @brief Updates the parameters of the node.
     *
     */
    void parameter_update_callback() {
        this->get_parameter("distance_threshold", distance_threshold_);
        this->get_parameter("linear_speed", linear_speed_);
        this->get_parameter("max_linear_speed", max_linear_speed_);
        this->get_parameter("angular_speed", angular_speed_);
        this->get_parameter("max_angular_speed", max_angular_speed_);
    }

    /**
     * @brief Callback for turtlesim pose.
     *
     */
    void turtlesim_pose_callback(const turtlesim::msg::Pose::SharedPtr msg) {
        turtlesim_current_pose_.x = msg->x;
        turtlesim_current_pose_.y = msg->y;
        turtlesim_current_pose_.theta = msg->theta;
    }

    /**
     * @brief Calculates distance from current pose to goal.
     *
     * @param target_pose Target position to reach.
     * @return double distance to goal.
     */
    double calculate_euclidean_distance(
        const turtlesim::msg::Pose &target_pose) {
        return std::sqrt(
            std::pow(target_pose.x - turtlesim_current_pose_.x, 2) +
            std::pow(target_pose.y - turtlesim_current_pose_.y, 2));
    }

    /**
     * @brief Check if turtlesim is close enough to the target.
     *
     * @param target_pose Target position to reach.
     * @return true if goal was reached, false otherwise.
     */
    bool goal_reached(const turtlesim::msg::Pose &target_pose) {
        auto distance = calculate_euclidean_distance(target_pose);
        RCLCPP_INFO(this->get_logger(),
                    "Starting goal_controller_action_server.");
        if (distance < distance_threshold_) {
            return true;
        }
        return false;
    }

    /**
     * @brief Updates speed values of the turtlesim.
     *
     * @param target_pose
     * @return distance to goal.
     */
    double move_to_goal(const turtlesim::msg::Pose &target_pose) {
        auto distance_to_goal = calculate_euclidean_distance(target_pose);
        RCLCPP_INFO(this->get_logger(), "Distance to goal: %f",
                    distance_to_goal);
        auto steering_angle =
            std::atan2(target_pose.y - turtlesim_current_pose_.y,
                       target_pose.x - turtlesim_current_pose_.x);
        auto angular_vel = steering_angle - turtlesim_current_pose_.theta;
        // Convert angular poses
        if (angular_vel > M_PI) {
            angular_vel = angular_vel - (2 * M_PI);
        }
        if (angular_vel <= -M_PI) {
            angular_vel = angular_vel + (2 * M_PI);
        }
        angular_vel *= angular_speed_;

        auto msg = std::make_unique<geometry_msgs::msg::Twist>();
        // Set Linear speed. Limit to MAX SPEED.
        msg->linear.x = linear_speed_ * distance_to_goal;
        msg->linear.x = std::min(max_linear_speed_, msg->linear.x);
        msg->linear.y = 0.0;
        msg->linear.z = 0.0;

        // Set Angular speed.
        msg->angular.x = 0.0;
        msg->angular.y = 0.0;
        msg->angular.z = angular_vel;
        msg->angular.z = std::min(max_angular_speed_, msg->angular.z);
        turtlesim_cmd_vel_pub_->publish(std::move(msg));
        return distance_to_goal;
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const MoveRobot::Goal> goal) {
        (void)uuid;
        auto target_position = goal->target_pos;
        RCLCPP_INFO(this->get_logger(),
                    "Received goal request with position (%f, %f, %f).",
                    target_position.x, target_position.y,
                    target_position.theta);
        if (goal_reached(target_position)) {
            RCLCPP_WARN(this->get_logger(),
                        "Rejecting goal because is below distance threshold.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMoveRobot> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal.");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(
        const std::shared_ptr<GoalHandleMoveRobot> goal_handle) {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin
        // up a new thread
        std::thread{std::bind(&GoalControllerServer::execute, this, _1),
                    goal_handle}
            .detach();
    }

    void execute(const std::shared_ptr<GoalHandleMoveRobot> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing goal.");
        rclcpp::Rate loop_rate(10);
        const auto goal = goal_handle->get_goal()->target_pos;
        auto feedback = std::make_shared<MoveRobot::Feedback>();
        auto &distance_to_goal = feedback->distance_to_goal;
        auto result = std::make_shared<MoveRobot::Result>();

        while (true) {
            // Check if there is a cancel request
            if (goal_handle->is_canceling()) {
                result->success = false;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled.");
                return;
            }

            // Check if goal is done
            if (goal_reached(goal)) {
                RCLCPP_INFO(this->get_logger(),
                            "Goal reached target position.");
                result->success = true;
                goal_handle->succeed(result);
                return;
            }

            // Move to towards goal.
            distance_to_goal = move_to_goal(goal);
            // Publish feedback
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Publish feedback.");

            loop_rate.sleep();
        }
    }
};  // class GoalControllerServer

}  // namespace goal_controller

RCLCPP_COMPONENTS_REGISTER_NODE(goal_controller::GoalControllerServer)
