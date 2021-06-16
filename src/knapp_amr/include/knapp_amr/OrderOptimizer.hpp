/**
 * @file OrderOptimizer.hpp
 * @author Gonzalo Cervetti (cervetti.g@gmail.com)
 * @brief Solution Node for the Knapp challenge
 * @version 0.1
 * @date 2020-12-29
 *
 * @copyright Copyright (c) 2020
 *
 */

#pragma once

// Standard libraries
#include <memory>

// ROS libraries
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "knapp_interfaces/msg/next_order.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// Knapp libraries
#include "knapp_amr/ConfigurationFile.hpp"
#include "knapp_amr/KnappTypes.hpp"
#include "knapp_amr/OrderFinder.hpp"
#include "knapp_amr/pathFinder/PathFinderInterface.hpp"

namespace knapp_amr {
class OrderOptimizer : public rclcpp::Node {
   public:
    /**
     * @brief CConstructor of the class
     *
     */
    OrderOptimizer(std::shared_ptr<PathFinderInterface> path_finder);

   private:
    void currentPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void nextOrderCallback(const knapp_interfaces::msg::NextOrder::SharedPtr msg) const;

    void orderFoundCallback(const Order& order);

    /**
     * @brief Retrieves all the parts required for the required order
     *
     * @param order Order to look for.
     * @return std::vector<Part> List of parts required for the order.
     */
    std::vector<Part> getPartsFromOrder(const Order& order);

    /**
     * @brief Publishes marker arrays for AMR as well as pick-ups.
     * 
     * @param parts elements to show in visualization array.
     */
    void publishMarkerArray(const std::vector<Part>& parts);

    // Subscriber for current position.
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pos_sub_;

    // Subscriber for current position.
    rclcpp::Subscription<knapp_interfaces::msg::NextOrder>::SharedPtr next_order_sub_;

     // Subscriber for current position.
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;   

    /// @brief Path where the yamls are stored.
    std::string config_path_;

    /// @brief represents the configuration file.
    std::shared_ptr<ConfigurationFile> configuration_file_;

    /// @brief list of products.
    ProductMap products_;

    /// @brief Finds the right order in files.
    std::shared_ptr<OrderFinder> order_finder_;

    std::shared_ptr<PathFinderInterface> path_finder_;

    /// @brief Current position of the robot.
    double current_pos_x_;
    double current_pos_y_;

    /// @brief ID count for elements placed in visualization msgs.
    int32_t id_count_;
};
}  // namespace knapp_amr