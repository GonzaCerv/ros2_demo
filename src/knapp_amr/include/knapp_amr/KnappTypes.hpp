/**
 * @file KnappTypes.hpp
 * @author Gonzalo Cervetti (cervetti.g@gmail.com)
 * @brief Defines types required for the challenge
 * @version 0.1
 * @date 2021-01-06
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once

// Standard libraries
#include <cstdint>
#include <string>
#include <vector>

namespace knapp_amr {
/**
 * @brief Represents one element of a product.
 *
 */
struct Part {
    /// @brief Name of the part
    std::string name;

    /// @brief Name of parent that belongs.
    std::string product_parent_name;

    /// @brief Coordinate X.
    double x;

    /// @brief Coordinate X.
    double y;
};

/**
 * @brief Product listed in the product file.
 *
 */
struct Product {
    /// @brief Name of the part.
    std::string name;

    /// @brief Id of the product.
    uint32_t id;

    /// @brief Id of the parent order
    uint32_t order_number_parent;

    /// @brief elements of a product.
    std::vector<Part> parts;
};

/**
 * @brief Order listed in the Order file.
 *
 */
struct Order {
    /// @brief Id of the order.
    uint32_t order_number;

    /// @brief Coordinate X.
    double x;

    /// @brief Coordinate X.
    double y;

    /// @brief products of a order.
    std::vector<uint32_t> products;
};
}  // namespace knapp_amr