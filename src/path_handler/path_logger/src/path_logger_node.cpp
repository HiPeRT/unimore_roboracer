#include <adx_data_ros/adx_data_ros.hpp>

#include "path_logger/path_logger_node.hpp"

PathLoggerNode::PathLoggerNode()
  : mPosArrived(false)
  , mSpeedArrived(false)
{
    mNode = std::make_shared<rclcpp::Node>("path_logger_node");

    int tmp;
    mNode->declare_parameter("sensor_mode", 0);
    mNode->declare_parameter("path_mode", 0);
    mNode->declare_parameter("filename", 0);
    mNode->declare_parameter("fixed_frame", 0);

    mNode->get_parameter<int>("sensor_mode", tmp);
    mSensorMode = static_cast<SENSOR_MODE>(tmp);
    mNode->get_parameter<int>("path_mode", tmp);
    mPathMode = static_cast<PATH_MODE>(tmp);
    mNode->get_parameter<std::string>("filename", mFilename);
    mNode->get_parameter<std::string>("fixed_frame", mFixedFrame);

    changeFile();

    switch (mSensorMode) {
        case SENSOR_MODE::NAVSAT: {
            mNavsatSub = mNode->create_subscription<sensor_msgs::msg::NavSatFix>(
              "/navsat",
              1,
              std::bind(&PathLoggerNode::navsatCallback, this, std::placeholders::_1));
            break;
        }
        case SENSOR_MODE::GPS: {
#ifdef USE_GPS
            mGpsSub = mNode->create_subscription<gps_msgs::msg::GPSFix>(
              "/gps", 1, std::bind(&PathLoggerNode::gpsCallback, this, std::placeholders::_1));
            break;
#else
            std::cerr << "unavailable" << std::endl;
            exit(-1);
#endif
        }
        case SENSOR_MODE::ODOM:
            mOdomVelSub = mNode->create_subscription<nav_msgs::msg::Odometry>(
              "/odom_vel",
              1,
              std::bind(&PathLoggerNode::twistCallback, this, std::placeholders::_1));

            // fall through
        case SENSOR_MODE::ODOM_NO_VEL: {
            mOdomPosSub = mNode->create_subscription<nav_msgs::msg::Odometry>(
              "/odom_pos",
              1,
              std::bind(&PathLoggerNode::positionCallback, this, std::placeholders::_1));
            break;
        }
        default:
            break;
    }

    mPathPub = mNode->create_publisher<nav_msgs::msg::Path>("/travelled_path", 10);

    rclcpp::spin(mNode);
}

PathLoggerNode::~PathLoggerNode()
{
    rclcpp::shutdown();
}

void PathLoggerNode::positionCallback(
  std::shared_ptr<nav_msgs::msg::Odometry> aOdomPosMsg)
{
    mPosArrived = true;
    adx::data::fromRos(aOdomPosMsg->pose.pose, mCurrentOdom);

    if (mSensorMode == SENSOR_MODE::ODOM_NO_VEL)
        mSpeedArrived = true;

    if (mPosArrived && mSpeedArrived) {
        mPosArrived = false;
        mSpeedArrived = false;
        log_path(mCurrentOdom);
    }
}

void PathLoggerNode::twistCallback(std::shared_ptr<nav_msgs::msg::Odometry> aOdomVelMsg)
{
    mSpeedArrived = true;
    adx::data::fromRos(aOdomVelMsg->twist.twist.linear, mCurrentOdom.linear);

    if (mPosArrived && mSpeedArrived) {
        mPosArrived = false;
        mSpeedArrived = false;
        log_path(mCurrentOdom);
    }
}

void PathLoggerNode::navsatCallback(
  std::shared_ptr<sensor_msgs::msg::NavSatFix> aNavSatMsg)
{
    adx::data::fromRos(aNavSatMsg->header.stamp, mCurrentOdom.timestamp);
    mCurrentOdom.position.x() = EARTH_RADIUS * (aNavSatMsg->longitude * M_PI / 180) *
                                 std::cos(aNavSatMsg->latitude * M_PI / 180);
    mCurrentOdom.position.y() = EARTH_RADIUS * (aNavSatMsg->latitude * M_PI / 180);
    mCurrentOdom.position.z() = aNavSatMsg->altitude;

    log_path(mCurrentOdom);
}

#ifdef USE_GPS
void PathLoggerNode::gpsCallback(std::shared_ptr<gps_msgs::msg::GPSFix> aGpsMsg)
{
    adx::data::fromRos(aGpsMsg->header.stamp,mCurrentOdom.timestamp);
    mCurrentOdom.position.x() =
      EARTH_RADIUS * (aGpsMsg->longitude * M_PI / 180) * std::cos(aGpsMsg->latitude * M_PI / 180);
    mCurrentOdom.position.y() = EARTH_RADIUS * (aGpsMsg->latitude * M_PI / 180);
    mCurrentOdom.position.z() = aGpsMsg->altitude;

    log_path(mCurrentOdom);
}
#endif

void PathLoggerNode::publishPath()
{
    nav_msgs::msg::Path path;
    path.header.stamp = mNode->get_clock()->now();
    path.header.frame_id = mFixedFrame;

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = path.header.stamp;
    pose.header.frame_id = mFixedFrame;

    adx::data::toRos(mCurrentOdom, pose.pose);

    path.poses.push_back(pose);
    mPathPub->publish(path);
}