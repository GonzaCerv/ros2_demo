/**
 * @file Configuration.hpp
 * @author Gonzalo Cervetti (cervetti.g@gmail.com)
 * @brief Represents the information from a configuration file.
 * @version 0.1
 * @date 2021-01-06
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once

// Standard libraries
#include <fstream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

// Knapp libraries
#include <yaml-cpp/yaml.h>

#include "knapp_amr/KnappTypes.hpp"

namespace knapp_amr {
using ProductMap = std::map<uint32_t, Product>;
class ConfigurationFile {
   public:
    /**
     * @brief Construct a new Configuration File object
     *
     * @param filename Location of the file that represents
     */
    explicit ConfigurationFile(const std::string &filename) : path_{filename} {};

    /**
     * @brief Returns a list of products listed in the file.
     *
     * @param filename name of the file to read. Null if the filename was provided in constructor.
     * @return std::vector<Product>
     */
    ProductMap getProducts() {
        // Check if filename is valid.
        if (path_.empty()) {
            throw std::runtime_error("Filename cannot be empty");
        }

        /// @brief products of this file.
        ProductMap response;
        YAML::Node product_node = YAML::LoadFile(path_);
        for (auto element : product_node) {
            Product product;
            product.id = element["id"].as<uint32_t>();
            product.name = element["id"].as<std::string>();
            for (auto part : element["parts"]) {
                Part prt;
                prt.name = part["part"].as<std::string>();
                prt.x = part["cx"].as<double>();
                prt.y = part["cy"].as<double>();
                prt.product_parent_name = product.name;
                product.parts.emplace_back(prt);
            }
            response[product.id] = product;
        }
        return response;
    }

   private:
    /// @brief path of the file to read.
    const std::string path_;
};
}  // namespace knapp_amr