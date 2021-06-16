/**
 * @file OrderFinder.hpp
 * @author Gonzalo Cervetti (cervetti.g@gmail.com)
 * @brief This class manages the search for orders in yaml files
 * @version 0.1
 * @date 2021-01-06
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once

// Standard libraries
#include <fstream>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// Knapp libraries
#include "knapp_amr/KnappTypes.hpp"

namespace knapp_amr {
using OrderFinderCallback = std::function<void(const Order&)>;
class OrderFinder {
   public:
    /**
     * @brief Construct a new Configuration File object.
     *
     * @param path Location where orders are stored.
     */
    explicit OrderFinder(const std::string& path, const OrderFinderCallback& callback);

    /**
     * @brief Finds the required order in the yaml folder.
     *
     * @param order_id Number of order to search.
     * @return std::shared_ptr<Order> Order when found, nullptr otherwise.
     */
    void findOrder(uint32_t order_id);

   private:
    /**
     * @brief Searches all the order files that are stored in the folder.
     *
     * @return std::vector<std::string> vector with the path of the files stored in orders folder.
     */
    std::vector<std::string> listOrderFiles();

    /// @brief Path where yamls are located.
    std::string path_;

    /// @brief Callback for returning orders found.
    OrderFinderCallback order_finder_callback_;
};
}  // namespace knapp_amr