///**
// * @page path_logger ROS node to log traveled paths
// *
// * `path_logger` provides a ROS1 and ROS2 nodes to save traveled paths at runtime.
// *
// * It is composed of:
// * - path_logger library:
// *   - PathLogger
// *   - include/path_logger.hpp
// * - path_logger ros node:
// *   - PathLoggerNode
// *   - include/path_logger_node.hpp
// */

/**
 * @file path_logger.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-05-24
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef PATH_LOGGER_PATH_LOGGER_HPP
#define PATH_LOGGER_PATH_LOGGER_HPP

#include <fstream>

#include <adx_data/odometry.hpp>

/**
 *  The average radius of the earth in meters
 */
#define EARTH_RADIUS 6.371e6

/**
 * Configure which sensor to log position from.
 */
enum class SENSOR_MODE
{
    ODOM = 0,        /*!< records Odometry messages */
    ODOM_NO_VEL = 1, /*!< records Odometry messages */
    NAVSAT = 2,      /*!< records NavSatFix messages */
    GPS = 3          /*!< records GPSFix messages */
};

/**
 * @enum PATH_MODE
 *
 * @brief Path saving mode
 *
 * Configure what information will be saved in the csv.
 */
enum class PATH_MODE
{
    PLAN = 0,       /*!< saves 3D position and 3D linear velocity */
    TRAJECTORY = 1, /*!< saves 3D position and 3D orientation */
    BILLBOARDS = 2  /*!< publishes poses as paths for visualization */
};

/**
 * @brief path logger class
 *
 * This class performs the actual logging independently from ROS.
 */
class PathLogger
{
  public:
    PathLogger();
    ~PathLogger();

  protected:
    bool mInitialized = false;

    SENSOR_MODE mSensorMode = static_cast<SENSOR_MODE>(0);
    PATH_MODE mPathMode = static_cast<PATH_MODE>(0);
    adx::data::Odometry mLastOdom;

    std::string mFilename;
    std::ofstream mTrjFile;

    void log_path(const adx::data::Odometry& aOdomMsg);
    void write_header();
    void write_last_odom();

    void changeFile();

    virtual void publishPath() = 0;
};

#endif // PATH_LOGGER_PATH_LOGGER_HPP
