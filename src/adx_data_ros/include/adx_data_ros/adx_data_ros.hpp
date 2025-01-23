#ifndef ADX_DATA_ROS_HPP
#define ADX_DATA_ROS_HPP

#include <adx_data/covariance.hpp>
#include <adx_data/header.hpp>
#include <adx_data/obstacle.hpp>
#include <adx_data/odometry.hpp>
#include <adx_data/path.hpp>
#include <adx_data/point.hpp>
#include <adx_data/pose.hpp>
#include <adx_data/quaternion.hpp>
#include <adx_data/timestamp.hpp>
#include <adx_data/twist.hpp>
#include <adx_data/vector.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/header.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <adx_msgs/msg/plan.hpp>
#include <builtin_interfaces/msg/time.hpp>

namespace adx {
namespace data {

// ------------------------------ Covariance--------------------------------
template<typename Type, unsigned long Size>
void toRos(const adx::data::Covariance<Type, Size>& src, std::array<double, Size*Size>& dst)
{
    // dst.resize(Size * Size);
    for (unsigned long col = 0; col < Size; col++) {
        for (unsigned long row = 0; row < Size; row++) {
            dst[col * Size + row] = static_cast<Type>(src(row, col));
        }
    }
}


template<typename Type, unsigned long Size>
void fromRos(const std::array<double, Size*Size>& src, adx::data::Covariance<Type, Size>& dst)
{
    for (unsigned long col = 0; col < Size; col++) {
        for (unsigned long row = 0; row < Size; row++) {
            dst(row, col) = static_cast<double>(src[col * Size + row]);
        }
    }
}


// ------------------------------Timestamp--------------------------------
void toRos(const adx::data::TimeStamp& src, builtin_interfaces::msg::Time& dst)
{

    dst = rclcpp::Time((rcl_time_point_value_t)(src.timestamp.time_since_epoch().count() / 1e9));
    // roscomp::time::from_sec(
    //   static_cast<double>(std::chrono::time_point_cast<std::chrono::nanoseconds>(timestamp)
    //                         .time_since_epoch()
    //                         .count()) /
    //   1e9);
}


void fromRos(const builtin_interfaces::msg::Time& src, adx::data::TimeStamp& dst)
{
    //dst = std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds>(std::chrono::nanoseconds(src.nanoseconds()));
    dst = std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds>(std::chrono::nanoseconds(static_cast<long>(src.sec * 1e9 + src.nanosec)));
}

// -----------------------------Header--------------------------------
void toRos(const adx::data::Header& src, std_msgs::msg::Header& dst)
{
    dst.frame_id = src.frame_id;
    toRos(src.timestamp, dst.stamp);
}


void fromRos(const std_msgs::msg::Header& src, adx::data::Header& dst)
{
    dst.frame_id = src.frame_id;
    fromRos(src.stamp, dst.timestamp);
}

// ------------------------------Vector--------------------------------
//toRos
template<typename Type, unsigned long Size>
void toRos(const adx::data::Vector<Type, Size>& src, geometry_msgs::msg::Vector3& dst)
{
    dst.x = src.x();
    dst.y = src.y();
    if constexpr (Size == 3) {
        dst.z = src.z();
    }
}

// fromRos
template<typename Type, unsigned long Size>
void fromRos(const geometry_msgs::msg::Vector3& src, adx::data::Vector<Type, Size>& dst)
{
    dst.x() = src.x;
    dst.y() = src.y;
    if constexpr (Size == 3) {
        dst.z() = src.z;
    }
}

// ------------------------------ Point--------------------------------
template<typename Type, unsigned long Size>
void toRos(const adx::data::Point<Type, Size>& src, geometry_msgs::msg::Point& dst)
{
    dst.x = src.position.x();
    dst.y = src.position.y();
    if constexpr (Size == 3) {
        dst.z = src.position.z();
    } else {
        dst.z = 0.0;
    }
}


template<typename Type, unsigned long Size>
void fromRos(const geometry_msgs::msg::Point& src, adx::data::Point<Type, Size>& dst)
{
    dst.position.x() = (double) src.x;
    dst.position.y() = (double) src.y;
    if constexpr (Size == 3) {
        dst.position.z() = (double) src.z;
    }
}


// ------------------------------  PoseOrientation--------------------------------
template<typename Type, unsigned long Size>
void toRos(const adx::data::internal::PoseOrientation<Type, Size>& src, geometry_msgs::msg::Quaternion& dst)
{
    if constexpr (Size == 2) {
        toRos(src.toQuaternion(), dst);
    }else {
        toRos(src.orientation, dst);
    }
}


template<typename Type, unsigned long Size>
void fromRos(const geometry_msgs::msg::Quaternion& src,  adx::data::internal::PoseOrientation<Type, Size>& dst)
{
    if constexpr (Size == 2) {
        adx::data::Quaternion<Type> tmp;
        fromRos(src, tmp);
        dst.yaw() = tmp.getRPY()[2];
    }else {
        fromRos(src, dst.orientation);
    }
}

// ------------------------------ Pose --------------------------------
// toRos
template<typename Type, unsigned long Size, bool HasHeader, bool HasCovariance>
void toRos(const adx::data::Pose<Type, Size, HasHeader, HasCovariance>& src, geometry_msgs::msg::Pose& dst)
{
    toRos(src, dst.position);
    toRos(src, dst.orientation);
}
// toRos SharedPtr
template<typename Type, unsigned long Size, bool HasHeader, bool HasCovariance>
void toRos(const adx::data::Pose<Type, Size, HasHeader, HasCovariance>& src, geometry_msgs::msg::Pose::SharedPtr& dst)
{
    toRos(src, dst->position);
    toRos(src, dst->orientation);
}

// fromRos
template<typename Type, unsigned long Size, bool HasHeader, bool HasCovariance>
void fromRos(const geometry_msgs::msg::Pose& src, adx::data::Pose<Type, Size, HasHeader, HasCovariance>& dst)
{
    fromRos(src.position, dst);
    fromRos(src.orientation, dst);
}
// fromRos SharedPtr
template<typename Type, unsigned long Size, bool HasHeader, bool HasCovariance>
void fromRos(const geometry_msgs::msg::Pose::SharedPtr& src, adx::data::Pose<Type, Size, HasHeader, HasCovariance>& dst)
{
    fromRos(src->position, dst);
    fromRos(src->orientation, dst);
}


// ------------------------------ Pose Covariance--------------------------------
// toRos
template<typename Type, unsigned long Size, bool HasHeader, bool HasCovariance>
void toRos(const adx::data::Pose<Type, Size, HasHeader, HasCovariance>& src, geometry_msgs::msg::PoseWithCovariance& dst)
{
    // TODO: compile time warning for header dropping
    toRos(src, dst.pose);
    toRos(src.pose_covariance, dst.covariance);
}
template<typename Type, unsigned long Size, bool HasHeader>
void toRos(const adx::data::Pose<Type, Size, HasHeader, false>& src, geometry_msgs::msg::PoseWithCovariance& dst)
{
    // TODO: compile time warning for header dropping
    // TODO: warn missing covariance information
    toRos(src, dst.pose);
}
// toRos SharedPtr
template<typename Type, unsigned long Size, bool HasHeader, bool HasCovariance>
void toRos(const adx::data::Pose<Type, Size, HasHeader, HasCovariance>& src, geometry_msgs::msg::PoseWithCovariance::SharedPtr& dst)
{
    // TODO: compile time warning for header dropping
    toRos(src, dst->pose);
    toRos(src.pose_covariance, dst->covariance);
}
template<typename Type, unsigned long Size, bool HasHeader>
void toRos(const adx::data::Pose<Type, Size, HasHeader, false>& src, geometry_msgs::msg::PoseWithCovariance::SharedPtr& dst)
{
    // TODO: compile time warning for header dropping
    // TODO: warn missing covariance information
    toRos(src, dst->pose);
}

// fromRos
template<typename Type, unsigned long Size, bool HasHeader, bool HasCovariance>
void fromRos(const geometry_msgs::msg::PoseWithCovariance& src, adx::data::Pose<Type, Size, HasHeader, HasCovariance>& dst)
{
    fromRos(src.pose, dst);
    fromRos(src.covariance, dst.pose_covariance);
}
template<typename Type, unsigned long Size, bool HasHeader>
void fromRos(const geometry_msgs::msg::PoseWithCovariance& src, adx::data::Pose<Type, Size, HasHeader, false>& dst)
{
    // TODO: warn losing covariance information
    fromRos(src.pose, dst);
}
// fromRos SharedPtr
template<typename Type, unsigned long Size, bool HasHeader, bool HasCovariance>
void fromRos(const geometry_msgs::msg::PoseWithCovariance::SharedPtr& src, adx::data::Pose<Type, Size, HasHeader, HasCovariance>& dst)
{
    fromRos(src->pose, dst);
    fromRos(src->covariance, dst.pose_covariance);
}
template<typename Type, unsigned long Size, bool HasHeader>
void fromRos(const geometry_msgs::msg::PoseWithCovariance::SharedPtr& src, adx::data::Pose<Type, Size, HasHeader, false>& dst)
{
    // TODO: warn losing covariance information
    fromRos(src->pose, dst);
}


// ------------------------------ Pose Stamped--------------------------------
// toRos
template<typename Type, unsigned long Size, bool HasHeader, bool HasCovariance>
void toRos(const adx::data::Pose<Type, Size, HasHeader, HasCovariance>& src, geometry_msgs::msg::PoseStamped& dst)
{
    // TODO: compile time warning for covariance dropping
    toRos(src, dst.pose);
    toRos(src, dst.header);
}
template<typename Type, unsigned long Size, bool HasCovariance>
void toRos(const adx::data::Pose<Type, Size, false, HasCovariance>& src, geometry_msgs::msg::PoseStamped& dst)
{
    // TODO: compile time warning for covariance dropping
    // TODO: warn missing header information
    toRos(src, dst.pose);
}

// toRos SharedPtr
template<typename Type, unsigned long Size, bool HasHeader, bool HasCovariance>
void toRos(const adx::data::Pose<Type, Size, HasHeader, HasCovariance>& src, geometry_msgs::msg::PoseStamped::SharedPtr& dst)
{
    // TODO: compile time warning for covariance dropping
    toRos(src, dst->pose);
    toRos(src, dst->header);
}
template<typename Type, unsigned long Size, bool HasCovariance>
void toRos(const adx::data::Pose<Type, Size, false, HasCovariance>& src, geometry_msgs::msg::PoseStamped::SharedPtr& dst)
{
    // TODO: compile time warning for covariance dropping
    // TODO: warn missing header information
    toRos(src, dst->pose);
}

// fromRos
template<typename Type, unsigned long Size, bool HasHeader, bool HasCovariance>
void fromRos(const geometry_msgs::msg::PoseStamped& src, adx::data::Pose<Type, Size, HasHeader, HasCovariance>& dst)
{
    fromRos(src.pose, dst);
    fromRos(src.header, dst);
}
template<typename Type, unsigned long Size, bool HasCovariance>
void fromRos(const geometry_msgs::msg::PoseStamped& src, adx::data::Pose<Type, Size, false, HasCovariance>& dst)
{
    // TODO: warn losing header information
    fromRos(src.pose, dst);
}

// fromRos SharedPtr
template<typename Type, unsigned long Size, bool HasHeader, bool HasCovariance>
void fromRos(const geometry_msgs::msg::PoseStamped::SharedPtr& src, adx::data::Pose<Type, Size, HasHeader, HasCovariance>& dst)
{
    fromRos(src->pose, dst);
    fromRos(src->header, dst);
}
template<typename Type, unsigned long Size, bool HasCovariance>
void fromRos(const geometry_msgs::msg::PoseStamped::SharedPtr& src, adx::data::Pose<Type, Size, false, HasCovariance>& dst)
{
    // TODO: warn losing header information
    fromRos(src->pose, dst);
}


// ------------------------------ PoseWithCovarianceStamped--------------------------------
// toRos
template<typename Type, unsigned long Size, bool HasHeader, bool HasCovariance>
void toRos(const adx::data::Pose<Type, Size, HasHeader, HasCovariance>& src, geometry_msgs::msg::PoseWithCovarianceStamped& dst)
{
    // TODO: compile time warning for covariance dropping
    toRos(src, dst.pose);
    toRos(src, dst.header);
}
template<typename Type, unsigned long Size, bool HasCovariance>
void toRos(const adx::data::Pose<Type, Size, false, HasCovariance>& src, geometry_msgs::msg::PoseWithCovarianceStamped& dst)
{
    // TODO: compile time warning for covariance dropping
    // TODO: warn missing header information 
    toRos(src, dst.pose);
}
// toRos SharedPtr
template<typename Type, unsigned long Size, bool HasHeader, bool HasCovariance>
void toRos(const adx::data::Pose<Type, Size, HasHeader, HasCovariance>& src, geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr& dst)
{
    // TODO: compile time warning for covariance dropping
    toRos(src, dst->pose);
    toRos(src, dst->header);
}
template<typename Type, unsigned long Size, bool HasCovariance>
void toRos(const adx::data::Pose<Type, Size, false, HasCovariance>& src, geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr& dst)
{
    // TODO: compile time warning for covariance dropping
    // TODO: warn missing header information 
    toRos(src, dst->pose);
}

// fromRos
template<typename Type, unsigned long Size, bool HasHeader, bool HasCovariance>
void fromRos(const geometry_msgs::msg::PoseWithCovarianceStamped& src, adx::data::Pose<Type, Size, HasHeader, HasCovariance>& dst)
{
    fromRos(src.pose, dst);
    fromRos(src.header, dst);
}
template<typename Type, unsigned long Size, bool HasCovariance>
void fromRos(const geometry_msgs::msg::PoseWithCovarianceStamped& src, adx::data::Pose<Type, Size, false, HasCovariance>& dst)
{
    // TODO: warn losing header information
    fromRos(src.pose, dst);
}
// fromRos SharedPtr
template<typename Type, unsigned long Size, bool HasHeader, bool HasCovariance>
void fromRos(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr& src, adx::data::Pose<Type, Size, HasHeader, HasCovariance>& dst)
{
    fromRos(src->pose, dst);
    fromRos(src->header, dst);
}
template<typename Type, unsigned long Size, bool HasCovariance>
void fromRos(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr& src, adx::data::Pose<Type, Size, false, HasCovariance>& dst)
{
    // TODO: warn losing header information
    fromRos(src->pose, dst);
}

// ------------------------------ Quaternion--------------------------------
// toRos
template<typename Type>
void toRos(const adx::data::Quaternion<Type>& src, geometry_msgs::msg::Quaternion& dst)
{
    dst.x = src.x();
    dst.y = src.y();
    dst.z = src.z();
    dst.w = src.w();
}

// fromRos
template<typename Type>
void fromRos(const geometry_msgs::msg::Quaternion& src, adx::data::Quaternion<Type>& dst)
{
    dst.x() = src.x;
    dst.y() = src.y;
    dst.z() = src.z;
    dst.w() = src.w;
}
// ------------------------------ Twist--------------------------------
// toRos
template<typename Type, bool HasHeader, bool HasCovariance>
void toRos(const adx::data::Twist<Type, HasHeader, HasCovariance>& src, geometry_msgs::msg::Twist& dst)
{
    dst.linear.x = src.linear.x();
    dst.linear.y = src.linear.y();
    dst.linear.z = src.linear.z();

    dst.angular.x = src.angular.x();
    dst.angular.y = src.angular.y();
    dst.angular.z = src.angular.z();
}

// fromRos
template<typename Type, bool HasHeader, bool HasCovariance>
void fromRos(const geometry_msgs::msg::Twist& src, adx::data::Twist<Type, HasHeader, HasCovariance>& dst)
{
    dst.linear.x() = src.linear.x;
    dst.linear.y() = src.linear.y;
    dst.linear.z() = src.linear.z;

    dst.angular.x() = src.angular.x;
    dst.angular.y() = src.angular.y;
    dst.angular.z() = src.angular.z;
}

// ------------------------------ Twist stamped--------------------------------
// toRos
template<typename Type, bool HasHeader, bool HasCovariance>
void toRos(const adx::data::Twist<Type, HasHeader, HasCovariance>& src, geometry_msgs::msg::TwistStamped& dst)
{
    toRos(src, dst.header);
    toRos(src, dst.twist);
}

// fromRos
template<typename Type, bool HasHeader, bool HasCovariance>
void fromRos(const geometry_msgs::msg::TwistStamped& src, adx::data::Twist<Type, HasHeader, HasCovariance>& dst)
{
    fromRos(src.header, dst);
    toRos(src.twist, dst);
}

// ------------------------------ Twist with covariance--------------------------------
// toRos
template<typename Type, bool HasHeader, bool HasCovariance>
void toRos(const adx::data::Twist<Type, HasHeader, HasCovariance>& src, geometry_msgs::msg::TwistWithCovariance& dst)
{
    toRos(src.twist_covariance, dst.covariance);
    toRos(src, dst.twist);
}

//fromRos
template<typename Type, bool HasHeader, bool HasCovariance>
void fromRos(const geometry_msgs::msg::TwistWithCovariance& src, adx::data::Twist<Type, HasHeader, HasCovariance>& dst)
{
    fromRos(src.covariance, dst.twist_covariance);
    fromRos(src.twist, dst);
}

// ------------------------------Twist with covariance stamped--------------------------------
// toRos
template<typename Type, bool HasHeader, bool HasCovariance>
void toRos(const adx::data::Twist<Type, HasHeader, HasCovariance>& src, geometry_msgs::msg::TwistWithCovarianceStamped& dst)
{
    toRos(src, dst.twist);
    toRos(src, dst.header);
}

// fromRos
template<typename Type, bool HasHeader, bool HasCovariance>
void fromRos(const geometry_msgs::msg::TwistWithCovarianceStamped& src, adx::data::Twist<Type, HasHeader, HasCovariance>& dst)
{
    fromRos(src.twist, dst);
    fromRos(src.header, dst);
}

// ------------------------------ Path--------------------------------
void toRos(const adx::data::Path& src, nav_msgs::msg::Path& dst)
{
    toRos(src, dst.header);
    for (const auto& pose : src.poses) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        toRos(pose, pose_stamped);
        dst.poses.push_back(pose_stamped);
    }
}

void fromRos(const nav_msgs::msg::Path& src, adx::data::Path& dst)
{
    fromRos(src.header, dst);
    for (const auto& pose_stamped : src.poses) {
        adx::data::Path::PoseType pose;
        fromRos(pose_stamped.pose, pose);
        dst.poses.push_back(pose);
    }
}
// ----------------------------Obstacle--------------------------------
// TODO: do we have a ROS type for this/should we create one?

// ----------------------------Odometry--------------------------------
void toRos(const adx::data::Odometry& src, nav_msgs::msg::Odometry& dst)
{
    toRos(src, dst.header);
    toRos(src, dst.pose);
    toRos(src, dst.twist);
}

void fromRos(const nav_msgs::msg::Odometry& src, adx::data::Odometry& dst)
{
    fromRos(src.header, dst);
    fromRos(src.pose, dst);
    fromRos(src.twist, dst);
}

// ------------------------------ Plan --------------------------------
void toRos(const adx::data::Plan& src, adx_msgs::msg::Plan& dst)
{
    toRos(src, dst.header);
    dst.points.resize(src.positions.size());
    for (size_t i = 0; i < src.positions.size(); i++) {
        geometry_msgs::msg::Point point_msg;
        toRos(src.positions[i], point_msg);
        dst.points[i].position = point_msg;
        geometry_msgs::msg::Vector3 speed_msg;
        toRos(src.speeds[i], speed_msg);
        dst.points[i].speed = speed_msg;
    }
}

void toRos(const adx::data::Plan& src, nav_msgs::msg::Path& dst)
{
    toRos(src, dst.header);
    dst.poses.resize(src.positions.size());
    std::cerr << "[WARNING] Converting adx::data::Plan to nav_msgs::msg::Path drops speeds from the output path!" << std::endl;
    for (size_t i = 0; i < src.positions.size(); i++) {
        geometry_msgs::msg::PoseStamped pose_msg;
        toRos(src.positions[i], dst.poses[i].pose.position);
    }
}

void fromRos(const adx_msgs::msg::Plan& src, adx::data::Plan& dst)
{
    fromRos(src.header, dst);
    dst.positions.resize(src.points.size());
    dst.speeds.resize(src.points.size());
    for (size_t i = 0; i < src.points.size(); i++) {
        adx::data::Point3f point;
        fromRos(src.points[i].position, dst.positions[i]);
        adx::data::Vector3f speed;
        fromRos(src.points[i].speed, dst.speeds[i]);
    }
}

} // namespace adx
} // namespace data

#endif // ADX_DATA_ROS_HPP