/**
 * @file PathFinderInterface.hpp
 * @author Gonzalo Cervetti (cervetti.g@gmail.com)
 * @brief Interface that calculates the optimal path to search for parts/
 * @version 0.1
 * @date 2021-1-10
 *
 * @copyright Copyright (c) 2020
 *
 */

#pragma once

// Knapp libraries
#include "knapp_amr/KnappTypes.hpp"

namespace knapp_amr {
class PathFinderInterface {
   public:
    /** @brief Destructor */
    virtual ~PathFinderInterface() = default;

    /**
     * @brief Calculates the order in which points should be covered
     *
     * @param parts parts to look for.
     * @return std::vector<Position> Vector with the order of points.
     */
    virtual std::vector<Part> findPath(std::vector<Part> parts) const = 0;
};
}  // namespace knapp_amr
