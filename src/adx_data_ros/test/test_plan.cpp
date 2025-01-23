#include <gtest/gtest.h>
#include "adx_data_ros/adx_data_ros.hpp"
#include "helper_functions.hpp"


namespace adx {
namespace data {

void compare_plan(const adx_msgs::msg::Plan& ros_plan, const adx::data::Plan& adx_plan)
{
    compare_header(ros_plan.header, static_cast<Header>(adx_plan));

    for (size_t i = 0; i < adx_plan.positions.size(); ++i) {
        compare_position<float, 3>(ros_plan.points[i].position, adx_plan.positions[i].position);
        compare_velocity<float>(ros_plan.points[i].speed, adx_plan.speeds[i]);
    }
}

class PlanTest : public ::testing::Test
{
  protected:
    rclcpp::Node::SharedPtr mNode;

    void SetUp() override
    {
        int argc = 0;
        char** argv = 0;
        rclcpp::init(argc, argv);
        mNode = std::make_shared<rclcpp::Node>("plan_test");
    }

    void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(PlanTest, FromRosPlan)
{
    const float x = 1.0, y = 2.0, z = 3.0;
    adx_msgs::msg::Plan ros_plan;
    ros_plan.header.frame_id = "map";
    ros_plan.header.stamp = mNode->get_clock()->now();
    ros_plan.points.resize(10);
    for (size_t i = 0; i < ros_plan.points.size(); ++i) {
        ros_plan.points[i].position.x = x + i;
        ros_plan.points[i].position.y = y + i;
        ros_plan.points[i].position.z = z + i;
        ros_plan.points[i].speed.x = x - i;
        ros_plan.points[i].speed.y = y - i;
        ros_plan.points[i].speed.z = z - i;
    }
    std::cout << "Testing Plan fromRos" << std::endl;
    adx::data::Plan adx_plan;
    fromRos(ros_plan, adx_plan);
    compare_plan(ros_plan, adx_plan);
}

TEST_F(PlanTest, ToRosPlan)
{
    const float x = 1.0, y = 2.0, z = 3.0;
    adx::data::Plan adx_plan;
    adx_plan.timestamp = std::chrono::steady_clock::now();
    adx_plan.frame_id = "map";
    adx_plan.positions.resize(10);
    adx_plan.speeds.resize(10);
    for (size_t i = 0; i < adx_plan.positions.size(); ++i) {
        adx_plan.positions[i].position.x() = x + i;
        adx_plan.positions[i].position.y() = y + i;
        adx_plan.positions[i].position.z() = z + i;
        adx_plan.speeds[i].x() = x - i;
        adx_plan.speeds[i].y() = y - i;
        adx_plan.speeds[i].z() = z - i;
    }
    std::cout << "Testing Plan toRos" << std::endl;
    adx_msgs::msg::Plan ros_plan;
    toRos(adx_plan, ros_plan);
    compare_plan(ros_plan, adx_plan);
}

} // namespace data
} // namespace adx