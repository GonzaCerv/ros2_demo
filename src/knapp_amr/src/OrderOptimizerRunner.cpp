/**
 * @file OrderOptimizerRunner.cpp
 * @author Gonzalo Cervetti (cervetti.g@gmail.com)
 * @brief Runner script for OrderOptimizer node.
 * @version 0.1
 * @date 2020-12-29
 *
 * @copyright Copyright (c) 2020
 *
 */

// Standard libraries
#include <iostream>
#include <memory>

// ROS libraries
#include "rclcpp/rclcpp.hpp"

// knapp libraries
#include "knapp_amr/OrderOptimizer.hpp"
#include "knapp_amr/pathFinder/travelSalesman/PathFinderTSP.hpp"
#include "knapp_amr/pathFinder/PathFinderInterface.hpp"

using knapp_amr::OrderOptimizer;
using knapp_amr::PathFinderTSP;
using knapp_amr::PathFinderInterface;

int main(int argc, char** argv) {
    std::shared_ptr<PathFinderInterface> path_finder = std::make_shared<PathFinderTSP>();
    try {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<OrderOptimizer>(path_finder));
        rclcpp::shutdown();
    } catch (std::exception& e) {
        std::cerr << "Exception thrown!: " << e.what() << std::endl;
    }
    return EXIT_FAILURE;
}