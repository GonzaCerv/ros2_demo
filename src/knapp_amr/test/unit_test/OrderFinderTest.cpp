/**
 * @file OrderFinderTest.cpp
 * @author Gonzalo Cervetti (cervetti.g@gmail.com)
 * @brief Testing for OrderFinder class.
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
#include "knapp_amr/KnappTypes.hpp"
#include "knapp_amr/OrderFinder.hpp"

using namespace ::testing;  // NOLINT Special exception for readable tests
using namespace knapp_amr;

class OrderFinderTest : public Test {
   public:
    OrderFinderTest() {}

    void SetUp() override {
        callback = std::bind(&OrderFinderTest::finderCallback, this, std::placeholders::_1);
    }

    void finderCallback(const Order &order) {
        std::cout << "new order called" << std::endl;
        order_vector.push_back(order);
    }

    std::vector<Order> order_vector;
    std::shared_ptr<OrderFinder> uut;
    OrderFinderCallback callback;
};

/**
 * @brief Checks error when creating an empty object.
 *
 */
TEST_F(OrderFinderTest, CreateEmptyString) {
    // Assert.
    ASSERT_THROW(uut = std::make_shared<OrderFinder>("", callback), std::runtime_error);
}

/**
 * @brief Checks error when looking wrong folder.
 *
 */
TEST_F(OrderFinderTest, WrongFolder) {
    // Assert.
    ASSERT_THROW(uut = std::make_shared<OrderFinder>("/thisDoesNotExist", callback),
                 std::runtime_error);
}

/**
 * @brief Checks correct creationg of folder.
 *
 */
TEST_F(OrderFinderTest, CorrectFolder) {
    // Cronstruct.
    uut = std::make_shared<OrderFinder>("/home/docker/ws/src/knapp_amr/test/yaml/order", callback);

    // Assert.
    ASSERT_TRUE(uut);
}

/**
 * @brief Searches for an order in yaml.
 *
 */
TEST_F(OrderFinderTest, FindOrder) {
    // Construct.
    uut = std::make_shared<OrderFinder>("/home/docker/ws/src/knapp_amr/test/yaml/order", callback);

    // Assert.
    auto before = std::chrono::system_clock::now();
    uut->findOrder(1000001);
    auto after = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(after - before).count();
    std::cout << "Search for order took: " << std::to_string(duration) << " seconds"
              << std::endl;

    // Test order found.
    ASSERT_FALSE(order_vector.empty());
    ASSERT_EQ((uint32_t)1000001, order_vector[0].order_number);
    ASSERT_NEAR(748.944, order_vector[0].x, 0.01);
    ASSERT_NEAR(474.71707, order_vector[0].y, 0.01);
    ASSERT_EQ((uint32_t)902, order_vector[0].products[0]);
    ASSERT_EQ((uint32_t)293, order_vector[0].products[1]);
    ASSERT_EQ((uint32_t)142, order_vector[0].products[2]);
    ASSERT_EQ((uint32_t)56, order_vector[0].products[3]);
    ASSERT_EQ((uint32_t)894, order_vector[0].products[4]);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}