/**
 * @file OrderFinder.cpp
 * @author Gonzalo Cervetti (cervetti.g@gmail.com)
 * @brief Runner for the UART Bridge Node.
 * @version 0.1
 * @date 2020-01-03
 *
 * @copyright Copyright (c) 2020
 *
 */

// Standard libraries
#include <sys/stat.h>
#include <sys/types.h>

#include <filesystem>
#include <functional>
#include <iostream>
#include <memory>

// Knapp libraries
#include "knapp_amr/ConfigurationFile.hpp"
#include "knapp_amr/OrderFinder.hpp"
#include "yaml-cpp/yaml.h"

constexpr char PATH_PARAM_NAME[] = "path";

namespace knapp_amr {

OrderFinder::OrderFinder(const std::string& path, const OrderFinderCallback& callback)
    : path_{path}, order_finder_callback_{callback} {
    // Check if filename is valid.
    if (path_.empty()) {
        throw std::runtime_error("Path cannot be empty");
    }
    struct stat info;
    if (stat(path_.c_str(), &info) != 0) {
        throw std::runtime_error("Path does not exist.");
    }
}

void OrderFinder::findOrder(uint32_t order_id) {
    // Retrieve the list of yaml files.
    auto order_list = listOrderFiles();
    std::atomic_bool order_found{false};
    std::list<std::thread> runner_threads;

    // Emplace one lambda expression for each order file found.
    for (const auto& order_file : order_list) {
        runner_threads.emplace_back([this, &order_found, order_id, order_file] {
            YAML::Node order_node = YAML::LoadFile(order_file);
            auto order_it = order_node.begin();
            // If one of the thread finds the order, the others will stop.
            while ((!order_found) && (order_it != order_node.end())) {
                auto order_iterator_id = (*order_it)["order"].as<uint32_t>();
                if (order_iterator_id == order_id) {
                    std::cout << "Order found!" << std::endl;
                    Order response_order;
                    response_order.order_number = (*order_it)["order"].as<uint32_t>();
                    response_order.x = (*order_it)["cx"].as<double>();
                    response_order.y = (*order_it)["cy"].as<double>();
                    for (auto product : (*order_it)["products"]) {
                        response_order.products.emplace_back(product.as<uint32_t>());
                    }
                    order_found = true;
                    std::cout << "calling callback" << std::endl;
                    order_finder_callback_(response_order);
                }
                ++order_it;
            }
        });
    }

    // Launch threads.
    for (auto& runner : runner_threads) {
        runner.join();
    }
}

std::vector<std::string> OrderFinder::listOrderFiles() {
    std::vector<std::string> result;
    for (const auto& entry : std::filesystem::directory_iterator(path_)) {
        std::string entry_path = entry.path();
        // Add only yaml files.
        if (entry_path.find(".yaml") != std::string::npos) {
            result.emplace_back(entry_path);
            std::cout << "Found order file " << entry_path << std::endl;
        }
    }
    return result;
}
}  // namespace knapp_amr