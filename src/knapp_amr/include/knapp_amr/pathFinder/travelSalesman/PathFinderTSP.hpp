/**
 * @file PathFinderTSP.hpp
 * @author Gonzalo Cervetti (cervetti.g@gmail.com)
 * @brief Finds the path to reach all objects using Travel Salesman Problem.
 * @version 0.1
 * @date 2021-01-10
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once

// Standard libraries
#include <memory>

// Knapp libraries
#include "algorithms.h"
#include "knapp_amr/KnappTypes.hpp"
#include "knapp_amr/pathFinder/PathFinderInterface.hpp"

namespace knapp_amr {
class PathFinderTSP : public PathFinderInterface {
   public:
    /// @brief Default constructor.
    PathFinderTSP(){};

    /// @brief Default destructor.
    ~PathFinderTSP(){};

    std::vector<Part> findPath(std::vector<Part> parts) const override {
        tsp travel_salesman_(parts);
        return travel_salesman_.find_path();
    };
};
}  // namespace knapp_amr