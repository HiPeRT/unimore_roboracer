#ifndef PATH_LOGGER_PATH_LOGGER_NODE_HPP
#define PATH_LOGGER_PATH_LOGGER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#ifdef USE_GPS
#include <gps_msgs/msg/gps_fix.hpp>
#endif

#include "path_logger/path_logger.hpp"

/**
 * @brief ROS1/2 path_logger node
 *
 * This class includes the ros2 communication required for path_logger functionality.
 */
class PathLoggerNode : public PathLogger
{
  public:
    PathLoggerNode();
    ~PathLoggerNode();

  private:
    bool mPosArrived;
    bool mSpeedArrived;

    std::string mFixedFrame;

    adx::data::Odometry mCurrentOdom = {}; /*!< current odometry */

    rclcpp::Node::SharedPtr mNode;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mOdomPosSub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mOdomVelSub;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr mNavsatSub;
#ifdef USE_GPS
    rclcpp::Subscription<gps_msgs::msg::GPSFix>::SharedPtr mGpsSub;
#endif

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mPathPub;

    void positionCallback(std::shared_ptr<nav_msgs::msg::Odometry> aOdomPosMsg);
    void twistCallback(std::shared_ptr<nav_msgs::msg::Odometry> aOdomVelMsg);

    void navsatCallback(std::shared_ptr<sensor_msgs::msg::NavSatFix> aNavsatMsg);
#ifdef USE_GPS
    void gpsCallback(std::shared_ptr<gps_msgs::msg::GPSFix> aGpsMsg);
#endif

    virtual void publishPath() override;
};

#endif // PATH_LOGGER_PATH_LOGGER_NODE_HPP
