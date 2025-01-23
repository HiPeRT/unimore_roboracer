#ifndef INCLUDE_PUREPURSUIT_NODE_HPP
#define INCLUDE_PUREPURSUIT_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <adx_msgs/msg/plan.hpp>
#include <adx_msgs/msg/arbitrated_ackermann_drive.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/int8.hpp>

#include <adx_data_ros/adx_data_ros.hpp>

#include "purepursuit/purepursuit.hpp"

/**
 * This class implements a ros wrapper around the PurePursuit class.
 */
class PurePursuitNode : public PurePursuit
{
  private:
    /**
     * Subscriber node
     */
    rclcpp::Node::SharedPtr mNode;

    /**
     * Executor used by the sub_node.
     */
    rclcpp::executors::MultiThreadedExecutor mExecutor;

    /**
     * Callback groups
     */
    rclcpp::CallbackGroup::SharedPtr mPoseCallbackGroup, mPathToFollowCallbackGroup,
      mGlobalPlanCallbackGroup, mLocalPathCallbackGroup;

    /**
     * Pose subscriber
     */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mPoseSub;

    /**
     * Suggested path subscriber
     */
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr mSuggestedPathSub;

    /**
     * Global plan subscriber
     */
    rclcpp::Subscription<adx_msgs::msg::Plan>::SharedPtr mGlobalPlanSub;

    /**
     * Local path subscriber
     */
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr mLocalPathSub;

    /**
     * Drive parameters publisher
     */
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr mDrivePub;
    /**
     * Arbitrated drive parameters publisher
     */
    rclcpp::Publisher<adx_msgs::msg::ArbitratedAckermannDrive>::SharedPtr mArbitratedDrivePub;

    /**
     * Spline position debug marker
     */
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mSplinePosePub;

    /**
     * Spline target debug marker
     */
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mGoalPub;

    /**
     * Global coordinates fixed frame
     */
    std::string mFixedFrame;

  public:
    PurePursuitNode();

    /** ROS callbacks **/

    /**
     * Callback to set which path to follow
     *
     * @brief Local path callback
     * @param data the new path
     */
    void pathToFollowCallback(std::shared_ptr<std_msgs::msg::Int8> data);

    /**
     * Callback to set the current vehicle pose
     * @brief Pose callback
     * @param pose the new pose
     */
    void poseCallback(std::shared_ptr<nav_msgs::msg::Odometry> pose);

    /**
     * Callback to set the global plan. Overwrites the previous plan
     * @brief Local path callback
     * @param plan the new global plan
     */
    void globalPlanCallback(std::shared_ptr<adx_msgs::msg::Plan> plan);

    /**
     * Callback to set the local path. Overwrites the local path
     * @brief Local path callback
     * @param path the new path
     */
    void localPathCallback(std::shared_ptr<nav_msgs::msg::Path> path);

    /**
     * Publishes the current drive parameters into ROS
     * @brief Publish drive parameters
     */
    void publishDriveParams();

    /**
     * Publishes the purepursuit debug markers
     * @brief Publish debug markers
     */
    void publishDebugMarkers();
};

#endif // INCLUDE_PUREPURSUIT_NODE_HPP
