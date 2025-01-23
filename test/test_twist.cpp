#include <gtest/gtest.h>
#include "adx_data_ros/adx_data_ros.hpp"
#include "helper_functions.hpp"

namespace adx {
namespace data {


class TwistTest : public ::testing::Test
{
  protected:
    rclcpp::Node::SharedPtr mNode;

    void SetUp() override
    {
        int argc = 0;
        char** argv = 0;
        rclcpp::init(argc, argv);
        mNode = std::make_shared<rclcpp::Node>("Twist_test");
    }

    void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(TwistTest, TwistdStampCovCopy)
{
    test_twist<double, true, true>();
}
TEST_F(TwistTest, TwistdStampCopy)
{
    test_twist<double, true, false>();
}
TEST_F(TwistTest, TwistdCovCopy)
{
    test_twist<double, false, true>();
}
TEST_F(TwistTest, TwistdCopy)
{
    test_twist<double, false, false>();
}

TEST_F(TwistTest, TwistfStampCovCopy)
{
    test_twist<float, true, true>();
}
TEST_F(TwistTest, TwistfStampCopy)
{
    test_twist<float, true, false>();
}
TEST_F(TwistTest, TwistfCovCopy)
{
    test_twist<float, false, true>();
}
TEST_F(TwistTest, TwistfCopy)
{
    test_twist<float, false, false>();
}


TEST_F(TwistTest, FromRosTwist)
{
    const double x = 1.0, y = 2.0, z = 3.0;
    geometry_msgs::msg::Twist ros_twist;

    ros_twist.linear.x = x;
    ros_twist.linear.y = y;
    ros_twist.linear.z = z;

    ros_twist.angular.x = x;
    ros_twist.angular.y = y;
    ros_twist.angular.z = z;

    std::cout << "Testing Twist<double, true, true> Twist fromRos" << std::endl;
    Twist<double, true, true> adx_twistd3tt;
    fromRos(ros_twist, adx_twistd3tt);
    compare_velocity<double>(ros_twist.linear, adx_twistd3tt.linear);
    compare_velocity<double>(ros_twist.angular, adx_twistd3tt.angular);

    std::cout << "Testing Twist<float, true, true> Twist fromRos" << std::endl;
    Twist<float, true, true> adx_twistf3tt;
    fromRos(ros_twist, adx_twistf3tt);
    compare_velocity<float>(ros_twist.linear, adx_twistf3tt.linear);
    compare_velocity<float>(ros_twist.angular, adx_twistf3tt.angular);

    std::cout << "Testing Twist<double, false, true> Twist fromRos" << std::endl;
    Twist<double, false, true> adx_twistd3ft;
    fromRos(ros_twist, adx_twistd3ft);
    compare_velocity<double>(ros_twist.linear, adx_twistd3ft.linear);
    compare_velocity<double>(ros_twist.angular, adx_twistd3ft.angular);

    std::cout << "Testing Twist<float, false, true> Twist fromRos" << std::endl;
    Twist<float, false, true> adx_twistf3ft;
    fromRos(ros_twist, adx_twistf3ft);
    compare_velocity<float>(ros_twist.linear, adx_twistf3ft.linear);
    compare_velocity<float>(ros_twist.angular, adx_twistf3ft.angular);

    std::cout << "Testing Twist<double, true, false> Twist fromRos" << std::endl;
    Twist<double, true, false> adx_twistd3tf;
    fromRos(ros_twist, adx_twistd3tf);
    compare_velocity<double>(ros_twist.linear, adx_twistd3tf.linear);
    compare_velocity<double>(ros_twist.angular, adx_twistd3tf.angular);

    std::cout << "Testing Twist<float, true, false> Twist fromRos" << std::endl;
    Twist<float, true, false> adx_twistf3tf;
    fromRos(ros_twist, adx_twistf3tf);
    compare_velocity<float>(ros_twist.linear, adx_twistf3tf.linear);
    compare_velocity<float>(ros_twist.angular, adx_twistf3tf.angular);

    std::cout << "Testing Twist<double, false, false> Twist fromRos" << std::endl;
    Twist<double, false, false> adx_twistd3ff;
    fromRos(ros_twist, adx_twistd3ff);
    compare_velocity<double>(ros_twist.linear, adx_twistd3ff.linear);
    compare_velocity<double>(ros_twist.angular, adx_twistd3ff.angular);

    std::cout << "Testing Twist<float, false, false> Twist fromRos" << std::endl;
    Twist<float, false, false> adx_twistf3ff;
    fromRos(ros_twist, adx_twistf3ff);
    compare_velocity<float>(ros_twist.linear, adx_twistf3ff.linear);
    compare_velocity<float>(ros_twist.angular, adx_twistf3ff.angular);
}

TEST_F(TwistTest, ToRosTwist)
{
    const double x = 1.0, y = 2.0, z = 3.0;

    std::cout << "Testing Twist<double, true, true> toRos Twist" << std::endl;
    Twist<double, true, true> adx_twistd3tt;
    adx_twistd3tt.linear = { x, y, z };
    adx_twistd3tt.angular = { x, y, z };
    geometry_msgs::msg::Twist ros_twist_d3tt;
    toRos(adx_twistd3tt, ros_twist_d3tt);
    compare_velocity<double>(ros_twist_d3tt.linear, adx_twistd3tt.linear);
    compare_velocity<double>(ros_twist_d3tt.angular, adx_twistd3tt.angular);
    std::cout << "Testing Twist<float, true, true> toRos Twist" << std::endl;
    Twist<float, true, true> adx_twistf3tt;
    adx_twistf3tt.linear = { static_cast<float>(x), static_cast<float>(y), static_cast<float>(z) };
    adx_twistf3tt.angular = { static_cast<float>(x), static_cast<float>(y), static_cast<float>(z) };
    geometry_msgs::msg::Twist ros_twist_f3tt;
    toRos(adx_twistf3tt, ros_twist_f3tt);
    compare_velocity<float>(ros_twist_f3tt.linear, adx_twistf3tt.linear);
    compare_velocity<float>(ros_twist_f3tt.angular, adx_twistf3tt.angular);

    std::cout << "Testing Twist<double, false, true> toRos Twist" << std::endl;
    Twist<double, false, true> adx_twistd3ft;
    adx_twistd3ft.linear = { x, y, z };
    adx_twistd3ft.angular = { x, y, z };
    geometry_msgs::msg::Twist ros_twist_d3ft;
    toRos(adx_twistd3ft, ros_twist_d3ft);
    compare_velocity<double>(ros_twist_d3ft.linear, adx_twistd3ft.linear);
    compare_velocity<double>(ros_twist_d3ft.angular, adx_twistd3ft.angular);
    std::cout << "Testing Twist<float, false, true> toRos Twist" << std::endl;
    Twist<float, false, true> adx_twistf3ft;
    adx_twistf3ft.linear = { static_cast<float>(x), static_cast<float>(y), static_cast<float>(z) };
    adx_twistf3ft.angular = { static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)};
    geometry_msgs::msg::Twist ros_twist_f3ft;
    toRos(adx_twistf3ft, ros_twist_f3ft);
    compare_velocity<float>(ros_twist_f3ft.linear, adx_twistf3ft.linear);
    compare_velocity<float>(ros_twist_f3ft.angular, adx_twistf3ft.angular);

    std::cout << "Testing Twist<double, true, false> toRos Twist" << std::endl;
    Twist<double, true, false> adx_twistd3tf;
    adx_twistd3tf.linear = { x, y, z };
    adx_twistd3tf.angular = { x, y, z };
    geometry_msgs::msg::Twist ros_twist_d3tf;
    toRos(adx_twistd3tf, ros_twist_d3tf);
    compare_velocity<double>(ros_twist_d3tf.linear, adx_twistd3tf.linear);
    compare_velocity<double>(ros_twist_d3tf.angular, adx_twistd3tf.angular);
    std::cout << "Testing Twist<float, true, false> toRos Twist" << std::endl;
    Twist<float, true, false> adx_twistf3tf;
    adx_twistf3tf.linear = { static_cast<float>(x), static_cast<float>(y), static_cast<float>(z) };
    adx_twistf3tf.angular = { static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)};
    geometry_msgs::msg::Twist ros_twist_f3tf;
    toRos(adx_twistf3tf, ros_twist_f3tf);
    compare_velocity<float>(ros_twist_f3tf.linear, adx_twistf3tf.linear);
    compare_velocity<float>(ros_twist_f3tf.angular, adx_twistf3tf.angular);

    std::cout << "Testing Twist<double, false, false> toRos Twist" << std::endl;
    Twist<double, false, false> adx_twistd3ff;
    adx_twistd3ff.linear = { x, y, z };
    adx_twistd3ff.angular = { x, y, z };
    geometry_msgs::msg::Twist ros_twist_d3ff;
    toRos(adx_twistd3ff, ros_twist_d3ff);
    compare_velocity<double>(ros_twist_d3ff.linear, adx_twistd3ff.linear);
    compare_velocity<double>(ros_twist_d3ff.angular, adx_twistd3ff.angular);
    std::cout << "Testing Twist<float, false, false> toRos Twist" << std::endl;
    Twist<float, false, false> adx_twistf3ff;
    adx_twistf3ff.linear = { static_cast<float>(x), static_cast<float>(y), static_cast<float>(z) };
    adx_twistf3ff.angular = { static_cast<float>(x), static_cast<float>(y), static_cast<float>(z) };
    geometry_msgs::msg::Twist ros_twist_f3ff;
    toRos(adx_twistf3ff, ros_twist_f3ff);
    compare_velocity<float>(ros_twist_f3ff.linear, adx_twistf3ff.linear);
    compare_velocity<float>(ros_twist_f3ff.angular, adx_twistf3ff.angular);
}


} // namespace adx
} // namespace data

