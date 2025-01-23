#include <gtest/gtest.h>
#include "adx_data_ros/adx_data_ros.hpp"


namespace adx {
namespace data {


TEST(PointTest, FromRosPoint)
{
    geometry_msgs::msg::Point ros_point;

    ros_point.x = 1.0;
    ros_point.y = 2.0;
    ros_point.z = 3.0;

    Point3d adx_point3d;
    fromRos(ros_point, adx_point3d);
    EXPECT_EQ(ros_point.x, adx_point3d.position.x());
    EXPECT_EQ(ros_point.y, adx_point3d.position.y());
    EXPECT_EQ(ros_point.z, adx_point3d.position.z());

    Point2d adx_point2d;
    fromRos(ros_point, adx_point2d);
    EXPECT_EQ(ros_point.x, adx_point2d.position.x());
    EXPECT_EQ(ros_point.y, adx_point2d.position.y());


    Point3f adx_point3f;
    fromRos(ros_point, adx_point3f);
    EXPECT_EQ(static_cast<float>(ros_point.x), adx_point3f.position.x());
    EXPECT_EQ(static_cast<float>(ros_point.y), adx_point3f.position.y());
    EXPECT_EQ(static_cast<float>(ros_point.z), adx_point3f.position.z());


    Point2f adx_point2f;
    fromRos(ros_point, adx_point2f);
    EXPECT_EQ(static_cast<float>(ros_point.x), adx_point2f.position.x());
    EXPECT_EQ(static_cast<float>(ros_point.y), adx_point2f.position.y());

}

TEST(PointTest, ToRosPoint)
{
    geometry_msgs::msg::Point ros_point;

    Point3d adx_point3d;
    adx_point3d.position.x() = 1.0;
    adx_point3d.position.y() = 2.0;
    adx_point3d.position.z() = 3.0;

    toRos(adx_point3d, ros_point);
    EXPECT_EQ(ros_point.x, adx_point3d.position.x());
    EXPECT_EQ(ros_point.y, adx_point3d.position.y());
    EXPECT_EQ(ros_point.z, adx_point3d.position.z());

    Point2d adx_point2d;
    adx_point2d.position.x() = 1.0;
    adx_point2d.position.y() = 2.0;

    toRos(adx_point2d, ros_point);
    EXPECT_EQ(ros_point.x, adx_point2d.position.x());
    EXPECT_EQ(ros_point.y, adx_point2d.position.y());
    EXPECT_EQ(ros_point.z, 0.0);

    Point3f adx_point3f;
    adx_point3f.position.x() = 1.0;
    adx_point3f.position.y() = 2.0;
    adx_point3f.position.z() = 3.0;

    toRos(adx_point3f, ros_point);
    EXPECT_EQ(static_cast<float>(ros_point.x), adx_point3f.position.x());
    EXPECT_EQ(static_cast<float>(ros_point.y), adx_point3f.position.y());
    EXPECT_EQ(static_cast<float>(ros_point.z), adx_point3f.position.z());

    Point2f adx_point2f;
    adx_point2f.position.x() = 1.0;
    adx_point2f.position.y() = 2.0;

    toRos(adx_point2f, ros_point);
    EXPECT_EQ(static_cast<float>(ros_point.x), adx_point2f.position.x());
    EXPECT_EQ(static_cast<float>(ros_point.y), adx_point2f.position.y());
    EXPECT_EQ(static_cast<float>(ros_point.z), 0.0);
}



} //namespace adx
} //namespace data