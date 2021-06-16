/**
 * @file ConfigurationFile.cpp
 * @author Gonzalo Cervetti (cervetti.g@gmail.com)
 * @brief Testing for ConfigurationFile class.
 * @version 0.1
 * @date 2021-01-12
 *
 * @copyright Copyright (c) 2021
 *
 */

// Standard library
#include <gtest/gtest.h>

#include <chrono>
#include <memory>

// Knapp libraries
#include "knapp_amr/ConfigurationFile.hpp"
#include "knapp_amr/KnappTypes.hpp"

using namespace ::testing;  // NOLINT Special exception for readable tests
using namespace knapp_amr;

class ConfigurationFiletest : public Test {
   public:
    ConfigurationFiletest() {}
};

/**
 * @brief Checks error when creating an empty object.
 *
 */
TEST_F(ConfigurationFiletest, CreateEmptyString) {
    // Construct.
    ConfigurationFile uut("");

    // Assert.
    ASSERT_THROW(uut.getProducts(), std::runtime_error);
}

/**
 * @brief Checks error when creating an empty object.
 *
 */
TEST_F(ConfigurationFiletest, CreateWrongPath) {
    // Construct.
    ConfigurationFile uut("yaml/wrong_file.yaml");

    // Assert.
    ASSERT_THROW(uut.getProducts(), std::runtime_error);
}

/**
 * @brief Checks error when creating an empty object.
 *
 */
TEST_F(ConfigurationFiletest, CreateCorrectPath) {
    // Construct.
    ConfigurationFile uut("/home/docker/ws/src/knapp_amr/test/yaml/configuration/products.yaml");

    auto products = uut.getProducts();
    
    // Assert.
    // Size of the map.
    ASSERT_EQ(1000, static_cast<int>(products.size()));

    // Check on of the elements.
    auto prod = products[4];
    ASSERT_EQ(4u, prod.id);
    ASSERT_EQ("Part B", prod.parts[0].name);
    ASSERT_DOUBLE_EQ  (550.09924, prod.parts[0].x);
    ASSERT_DOUBLE_EQ  (655.423, prod.parts[0].y);
    ASSERT_EQ("4", prod.parts[0].product_parent_name);

}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}