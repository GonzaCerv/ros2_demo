/**
 * @file OrderOptimizer.cpp
 * @author Gonzalo Cervetti (cervetti.g@gmail.com)
 * @brief Runner for the UART Bridge Node.
 * @version 0.1
 * @date 2020-01-03
 *
 * @copyright Copyright (c) 2020
 *
 */

// Standard libraries
#include <algorithm>
#include <functional>
#include <iostream>
#include <memory>

// ROS libraries
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "knapp_interfaces/msg/next_order.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Knapp libraries
#include "knapp_amr/ConfigurationFile.hpp"
#include "knapp_amr/OrderFinder.hpp"
#include "knapp_amr/OrderOptimizer.hpp"
#include "knapp_amr/pathFinder/PathFinderInterface.hpp"

constexpr char PATH_PARAM_NAME[] = "path";

namespace knapp_amr {
OrderOptimizer::OrderOptimizer(std::shared_ptr<PathFinderInterface> path_finder)
    : Node("order_optimizer"), path_finder_{path_finder} {
    this->declare_parameter<std::string>("path", "not found");
    this->get_parameter("path", config_path_);
    RCLCPP_INFO(this->get_logger(), "path: %s", config_path_.c_str());

    // Subscribers.
    current_pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "currentPosition", 10,
        std::bind(&OrderOptimizer::currentPoseCallback, this, std::placeholders::_1));

    next_order_sub_ = this->create_subscription<knapp_interfaces::msg::NextOrder>(
        "nextOrder", 10,
        std::bind(&OrderOptimizer::nextOrderCallback, this, std::placeholders::_1));

    marker_array_pub_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("order_path", 10);

    // Parse the configuration file and get the list of products.
    configuration_file_ =
        std::make_shared<ConfigurationFile>(config_path_ + "configuration/products.yaml");
    products_ = configuration_file_->getProducts();

    // Object to find orders.
    order_finder_ = std::make_shared<OrderFinder>(
        config_path_ + "orders",
        std::bind(&OrderOptimizer::orderFoundCallback, this, std::placeholders::_1));

    current_pos_x_ = 0.0f;
    current_pos_y_ = 0.0f;
}

void OrderOptimizer::currentPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_pos_x_ = (double)msg->pose.position.x;
    current_pos_y_ = (double)msg->pose.position.y;
}

void OrderOptimizer::nextOrderCallback(
    const knapp_interfaces::msg::NextOrder::SharedPtr msg) const {
    // Find the order description in the files.
    std::cout << "looking for order " << std::to_string(msg->order_id) << std::endl;
    order_finder_->findOrder(msg->order_id);
}

void OrderOptimizer::orderFoundCallback(const Order &order) {
    std::cout << "Order found " << std::to_string(order.order_number) << std::endl;
    Product next_product;

    // Retrieve all parts that must be fetched.
    auto parts = getPartsFromOrder(order);
    uint32_t n_parts = parts.size();
    std::cout << "Size of parts: " << n_parts << std::endl;

    // Add a Part element that represents the position of the robot. This will allow to
    // optimize the path based on the current position.
    Part robot{"robot", "", current_pos_x_, current_pos_y_};
    parts.emplace_back(robot);

    // Find the shortest path that covers all parts.
    auto parts_path = path_finder_->findPath(parts);

    // Look the position of the robot inside the vector.
    uint32_t robot_position = 0;
    while (parts_path[robot_position].name != "robot") {
        ++robot_position;
    }
    // Rotate the vector. This sets the robot object as the first element of the vector.
    std::rotate(parts_path.begin(), parts_path.begin() + robot_position, parts_path.end());

    // Print all the elements appart from the first. This is the shortest path.
    for (uint32_t index = 1; index < parts_path.size(); ++index) {
        auto part = parts_path[index];
        std::cout << "Fetching " << part.name << " for product " << part.product_parent_name
                  << " at x: " << part.x << ", y: " << part.y << std::endl;
    }

    std::cout << "Delivering to destination x: " << order.x << ", y: " << order.y << std::endl;

    // Publish visualization array.
    publishMarkerArray(parts_path);

    // Update the current location of the robot.
    current_pos_x_ = order.x;
    current_pos_y_ = order.y;
}

void OrderOptimizer::publishMarkerArray(const std::vector<Part> &parts) {
    int32_t id_count = 0;
    visualization_msgs::msg::MarkerArray marker_array;
    auto clock = rclcpp::Clock();

    // Publish current position of the robot.
    visualization_msgs::msg::Marker robot_marker;
    robot_marker.header.frame_id = "base_link";
    robot_marker.header.stamp = clock.now();
    robot_marker.id = 0;
    robot_marker.type = visualization_msgs::msg::Marker::CUBE;
    robot_marker.action = visualization_msgs::msg::Marker::ADD;
    robot_marker.pose.position.x = current_pos_x_;
    robot_marker.pose.position.y = current_pos_y_;
    robot_marker.pose.position.z = 0;
    robot_marker.pose.orientation.x = 0.0;
    robot_marker.pose.orientation.y = 0.0;
    robot_marker.pose.orientation.z = 0.0;
    robot_marker.pose.orientation.w = 1.0;
    robot_marker.scale.x = 1;
    robot_marker.scale.y = 0.3;
    robot_marker.scale.z = 0.3;
    robot_marker.color.a = 1.0;
    robot_marker.color.r = 1.0;
    robot_marker.color.g = 0.0;
    robot_marker.color.b = 0.0;
    robot_marker.lifetime = rclcpp::Duration(5.0);
    marker_array.markers.push_back(robot_marker);

    // Add parts to visualization.
    for (auto &it : parts) {
        visualization_msgs::msg::Marker part_marker;
        part_marker.header.frame_id = it.name;
        part_marker.header.stamp = clock.now();
        part_marker.id = id_count;
        part_marker.type = visualization_msgs::msg::Marker::CYLINDER;
        part_marker.action = visualization_msgs::msg::Marker::ADD;
        part_marker.pose.position.x = it.x;
        part_marker.pose.position.y = it.y;
        part_marker.pose.position.z = 0;
        part_marker.pose.orientation.x = 0.0;
        part_marker.pose.orientation.y = 0.0;
        part_marker.pose.orientation.z = 0.0;
        part_marker.pose.orientation.w = 1.0;
        part_marker.scale.x = 1;
        part_marker.scale.y = 0.3;
        part_marker.scale.z = 0.3;
        part_marker.color.a = 1.0;
        part_marker.color.r = 0.0;
        part_marker.color.g = 1.0;
        part_marker.color.b = 0.0;
        robot_marker.lifetime = rclcpp::Duration(5.0);
        marker_array.markers.push_back(part_marker);
        ++id_count;
    }
    // Publish.
    marker_array_pub_->publish(marker_array);
}

std::vector<Part> OrderOptimizer::getPartsFromOrder(const Order &order) {
    std::vector<Part> parts;
    for (auto &prod : order.products) {
        for (auto &part : products_[prod].parts) {
            parts.emplace_back(part);
        }
    }
    return parts;
}

}  // namespace knapp_amr