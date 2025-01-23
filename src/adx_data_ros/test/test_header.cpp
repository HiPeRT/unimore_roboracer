#include <gtest/gtest.h>
#include "adx_data_ros/adx_data_ros.hpp"
#include "helper_functions.hpp"

namespace adx {
namespace data {



class HeaderTest : public ::testing::Test
{
  protected:
    void SetUp() override
    {
        int argc = 0;
        char** argv = 0;
        rclcpp::init(argc, argv);
        mNode = std::make_shared<rclcpp::Node>("dummy");
    }

    void TearDown() override { rclcpp::shutdown(); }

    rclcpp::Node::SharedPtr mNode;
};

TEST_F(HeaderTest, FromRos)
{
    std_msgs::msg::Header ros_header;

    ros_header.stamp = mNode->get_clock()->now();
    ros_header.frame_id = "map";

    Header adx_header;
    fromRos(ros_header, adx_header);
    compare_header(ros_header, adx_header);

    // EXPECT_EQ(adx_header.timestamp.timestamp.time_since_epoch().count()
    //           , static_cast<long>(ros_header.stamp.sec * 1e-9 + ros_header.stamp.nanosec));
    // EXPECT_EQ(adx_header.frame_id, ros_header.frame_id);
}

TEST_F(HeaderTest, ToRos)
{
    Header adx_header;

    adx_header.timestamp = std::chrono::steady_clock::now();
    adx_header.frame_id = "map";

    std_msgs::msg::Header ros_header;
    toRos(adx_header, ros_header);
    compare_header(ros_header, adx_header);

    // EXPECT_EQ(static_cast<long>(adx_header.timestamp.timestamp.time_since_epoch().count() * 1e-9)
    //           , static_cast<long>(ros_header.stamp.sec * 1e-9 + ros_header.stamp.nanosec));  // TODO: Check if correct, sec is always 0
    // EXPECT_EQ(ros_header.frame_id, adx_header.frame_id);
}

} // namespace data
} // namespace adx

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
