#ifndef ADX_DATA_TEST_POSE_HELPERS
#define ADX_DATA_TEST_POSE_HELPERS

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

#include "adx_data/pose.hpp"

namespace adx {
namespace data {

template<typename Type, unsigned long Size, bool HasHeader, bool HasCovariance>
void init_pose(Pose<Type, Size, HasHeader, HasCovariance>& adx_pose)
{
    const Type pos[3] = { 1.0, 2.0, 3.0 };
    const Type rot[3] = { 0.1, 0.2, 0.3 };
    const std::string frame = "map";

    if constexpr (HasHeader) {
        adx_pose.timestamp = std::chrono::steady_clock::now();
        std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds> tmp{};
        EXPECT_NE(adx_pose.timestamp.timestamp, tmp);

        adx_pose.frame_id = frame;
        EXPECT_EQ(adx_pose.frame_id, frame);
    }

    for (unsigned long i = 0; i < Size; ++i) {
        adx_pose.position(i) = pos[i];
        EXPECT_EQ(adx_pose.position(i), pos[i]);
    }

    if constexpr (Size == 2) {
        adx_pose.yaw() = rot[2];
        EXPECT_EQ(adx_pose.yaw(), rot[2]);
    } else if constexpr (Size == 3) {
        adx_pose.orientation.setRPY(rot[0], rot[1], rot[2]);

        Eigen::Quaternion<Type> q;
        q = Eigen::AngleAxis<Type>(rot[0], Eigen::Matrix<Type, 3, 1>::UnitX()) *
            Eigen::AngleAxis<Type>(rot[1], Eigen::Matrix<Type, 3, 1>::UnitY()) *
            Eigen::AngleAxis<Type>(rot[2], Eigen::Matrix<Type, 3, 1>::UnitZ());

        EXPECT_EQ(adx_pose.orientation.w(), q.w());
        EXPECT_EQ(adx_pose.orientation.x(), q.x());
        EXPECT_EQ(adx_pose.orientation.y(), q.y());
        EXPECT_EQ(adx_pose.orientation.z(), q.z());
    }

    if constexpr (HasCovariance) {
        const long int cov_rows = adx_pose.pose_covariance.rows();
        const long int cov_cols = adx_pose.pose_covariance.cols();
        for (long int i = 0; i < cov_rows; ++i) {
            for (long int j = 0; j < cov_cols; ++j) {
                adx_pose.pose_covariance(i * cov_cols + j) = static_cast<Type>(i * cov_cols + j);
                EXPECT_EQ(adx_pose.pose_covariance(i * cov_cols + j),
                          static_cast<Type>(i * cov_cols + j));
            }
        }
    }
}

template<typename Type, bool HasHeader, bool HasCovariance>
void init_twist(Twist<Type, HasHeader, HasCovariance>& adx_twist)
{
    const Type lin[3] = { 1.0, 2.0, 3.0 };
    const Type ang[3] = { 1.0, 2.0, 3.0 };
    const std::string frame = "map";

    if constexpr (HasHeader) {
        adx_twist.timestamp = std::chrono::steady_clock::now();
        std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds> tmp{};
        EXPECT_NE(adx_twist.timestamp.timestamp, tmp);

        adx_twist.frame_id = frame;
        EXPECT_EQ(adx_twist.frame_id, frame);
    }

    for (unsigned long i = 0; i < 3; ++i) {
        adx_twist.linear(i) = lin[i];
        adx_twist.angular(i) = ang[i];

        EXPECT_EQ(adx_twist.linear(i), lin[i]);
        EXPECT_EQ(adx_twist.angular(i), ang[i]);

    }

    if constexpr (HasCovariance) {
        const long int cov_rows = adx_twist.twist_covariance.rows();
        const long int cov_cols = adx_twist.twist_covariance.cols();
        for (long int i = 0; i < cov_rows; ++i) {
            for (long int j = 0; j < cov_cols; ++j) {
                adx_twist.twist_covariance(i * cov_cols + j) = static_cast<Type>(i * cov_cols + j);
                EXPECT_EQ(adx_twist.twist_covariance(i * cov_cols + j),
                          static_cast<Type>(i * cov_cols + j));
            }
        }
    }
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion& ros_quaternion)
{
    static double roll, pitch, yaw;
    tf2::Quaternion tf2_quaternion(
      ros_quaternion.x, ros_quaternion.y, ros_quaternion.z, ros_quaternion.w);
    tf2::Matrix3x3 tf2_rotation_matrix(tf2_quaternion);
    tf2_rotation_matrix.getRPY(roll, pitch, yaw);
    return yaw;
}

void compare_header(const std_msgs::msg::Header& ros_header, const adx::data::Header& adx_header)
{
    // convert the adx stamp to the same lenght of the ros one
    long ros_stamp = static_cast<long>(ros_header.stamp.sec * 1e-9 + ros_header.stamp.nanosec);
    double ros_stamp_digits = log10(ros_stamp) + 1;

    long adx_stamp = adx_header.timestamp.timestamp.time_since_epoch().count();
    double adx_stamp_digits = log10(adx_stamp) + 1;

    long divisor = pow(10, adx_stamp_digits - ros_stamp_digits);
    long comparable_adx_stamp = static_cast<long>(adx_stamp / divisor);

    EXPECT_EQ(ros_stamp, comparable_adx_stamp);
    EXPECT_EQ(ros_header.frame_id, adx_header.frame_id);
}

void compare_header(const adx::data::Header& adx_header1, const adx::data::Header& adx_header2)
{
    EXPECT_EQ(adx_header1.timestamp, adx_header2.timestamp);
    EXPECT_EQ(adx_header1.frame_id, adx_header2.frame_id);
}

template<typename Type, unsigned long Size>
void compare_position(const geometry_msgs::msg::Point& ros_point,
                      const adx::data::Vector<Type, Size>& adx_vector)
{
    const double* ros_point_ptr = &ros_point.x;

    for (unsigned int i = 0; i < Size; ++i) {
        EXPECT_EQ(static_cast<Type>(ros_point_ptr[i]), static_cast<double>(adx_vector(i)));
    }
}

template<typename Type, unsigned long Size>
void compare_position(const adx::data::Vector<Type, Size>& adx_vector1,
                      const adx::data::Vector<Type, Size>& adx_vector2)
{
    for (unsigned int i = 0; i < Size; ++i) {
        EXPECT_EQ(adx_vector1(i), adx_vector2(i));
    }
}
template<typename Type>
void compare_velocity(const geometry_msgs::msg::Vector3& ros_point,
                      const adx::data::Vector<Type, 3>& adx_vector)
{
    const double* ros_point_ptr = &ros_point.x;

    for (unsigned int i = 0; i < 3; ++i) {
        EXPECT_EQ(static_cast<Type>(ros_point_ptr[i]), static_cast<double>(adx_vector(i)));
    }
}

template<typename Type>
void compare_velocity(const adx::data::Vector<Type, 3>& adx_vector1,
                      const adx::data::Vector<Type, 3>& adx_vector2)
{
    for (unsigned int i = 0; i < 3; ++i) {
        EXPECT_EQ(adx_vector1(i), adx_vector2(i));
    }
}

template<typename Type>
void compare_orientation(const geometry_msgs::msg::Quaternion& ros_quaternion,
                         const adx::data::Quaternion<Type>& adx_quaternion)
{
    EXPECT_EQ(static_cast<Type>(ros_quaternion.w), adx_quaternion.w());
    EXPECT_EQ(static_cast<Type>(ros_quaternion.x), adx_quaternion.x());
    EXPECT_EQ(static_cast<Type>(ros_quaternion.y), adx_quaternion.y());
    EXPECT_EQ(static_cast<Type>(ros_quaternion.z), adx_quaternion.z());
}

template<typename Type>
void compare_orientation(const adx::data::Quaternion<Type>& adx_quaternion1,
                         const adx::data::Quaternion<Type>& adx_quaternion2)
{
    EXPECT_EQ(adx_quaternion1.w(), adx_quaternion2.w());
    EXPECT_EQ(adx_quaternion1.x(), adx_quaternion2.x());
    EXPECT_EQ(adx_quaternion1.y(), adx_quaternion2.y());
    EXPECT_EQ(adx_quaternion1.z(), adx_quaternion2.z());
}

template<typename Type, unsigned long Size>
void compare_covariance(const std::array<double, 36>& ros_covariance,
                        const adx::data::Covariance<Type, Size>& adx_covariance)
{
    for (unsigned long i = 0; i < Size * Size; ++i) {
        EXPECT_EQ(static_cast<Type>(ros_covariance[i]), adx_covariance(i % Size, i / Size));
    }
}

template<typename Type, unsigned long Size>
void compare_covariance(const adx::data::Covariance<Type, Size>& adx_covariance1,
                        const adx::data::Covariance<Type, Size>& adx_covariance2)
{
    for (unsigned long i = 0; i < Size * Size; ++i) {
        EXPECT_EQ(adx_covariance1(i % Size, i / Size), adx_covariance2(i % Size, i / Size));
    }
}

template<typename Type1, typename Type2>
void compare_yaw(const Type1& yaw1, const Type2& yaw2)
{
    EXPECT_NEAR(static_cast<Type2>(yaw1), static_cast<Type1>(yaw2), 1e-5);
}

template<typename Type, unsigned long Size, bool HasHeader, bool HasCovariance>
void compare_pose(const Pose<Type, Size, HasHeader, HasCovariance>& adx_pose1,
                  const Pose<Type, Size, HasHeader, HasCovariance>& adx_pose2)
{
    if constexpr (HasHeader) {
        compare_header(static_cast<Header>(adx_pose1), static_cast<Header>(adx_pose2));
    }

    compare_position<Type, Size>(adx_pose1.position, adx_pose2.position);

    if constexpr (Size == 2) {
        compare_yaw<Type, Type>(adx_pose1.yaw(), adx_pose2.yaw());
    } else if constexpr (Size == 3) {
        compare_orientation<Type>(adx_pose1.orientation, adx_pose2.orientation);
    }

    if constexpr (HasCovariance) {
        compare_covariance(adx_pose1.pose_covariance, adx_pose2.pose_covariance);
    }
}


template<typename Type,  bool HasHeader, bool HasCovariance>
void compare_twist(const Twist<Type, HasHeader, HasCovariance>& adx_twist1,
                   const Twist<Type, HasHeader, HasCovariance>& adx_twist2)
{
    if constexpr (HasHeader) {
        compare_header(static_cast<Header>(adx_twist1), static_cast<Header>(adx_twist2));
    }

    compare_velocity<Type>(adx_twist1.linear, adx_twist2.linear);
    compare_velocity<Type>(adx_twist1.angular, adx_twist2.angular);


    if constexpr (HasCovariance) {
        compare_covariance(adx_twist1.twist_covariance, adx_twist2.twist_covariance);
    }
}

template<typename Type, unsigned long Size, bool HasHeader, bool HasCovariance>
void test_pose()
{
    Pose<Type, Size, HasHeader, HasCovariance> adx_pose;
    init_pose<Type, Size, HasHeader, HasCovariance>(adx_pose);

    Pose<Type, Size, HasHeader, HasCovariance> adx_pose_constructor(adx_pose);
    compare_pose<Type, Size, HasHeader, HasCovariance>(adx_pose, adx_pose_constructor);

    Pose<Type, Size, HasHeader, HasCovariance> adx_pose_assignment;
    adx_pose_assignment = adx_pose;
    compare_pose<Type, Size, HasHeader, HasCovariance>(adx_pose, adx_pose_assignment);
}


template<typename Type, bool HasHeader, bool HasCovariance>
void test_twist()
{
    Twist<Type, HasHeader, HasCovariance> adx_twist;
    init_twist<Type, HasHeader, HasCovariance>(adx_twist);

    Twist<Type, HasHeader, HasCovariance> adx_twist_constructor(adx_twist);
    compare_twist<Type, HasHeader, HasCovariance>(adx_twist, adx_twist_constructor);

    Twist<Type, HasHeader, HasCovariance> adx_twist_assignment;
    adx_twist_assignment = adx_twist;
    compare_twist<Type, HasHeader, HasCovariance>(adx_twist, adx_twist_assignment);
}

} // namespace data
} // namespace adx

#endif // ADX_DATA_TEST_POSE_HELPERS