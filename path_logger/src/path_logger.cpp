#include <cassert>
#include <iomanip>
#include <iostream>
#define _USE_MATH_DEFINES
#include <cmath>

#include "path_logger/path_logger.hpp"

PathLogger::PathLogger() {}

PathLogger::~PathLogger()
{
    mTrjFile.close();
}

void PathLogger::changeFile()
{
    std::cout << "opening " << mFilename << std::endl;

    if (mTrjFile.is_open())
        mTrjFile.close();

    mTrjFile.open(mFilename, std::ofstream::out);

    if (!mTrjFile.good()) {
        std::cerr << "bad file" << std::endl;
        exit(-1);
    }
}

void PathLogger::log_path(const adx::data::Odometry& odom_data)
{
    if (!mInitialized) {
        mLastOdom = odom_data;
        mInitialized = true;
        write_header();
        std::cout << "initialized" << std::endl;
        return;
    }

    if (odom_data.timestamp < mLastOdom.timestamp)
        return; // rollback
    mLastOdom.timestamp = odom_data.timestamp;

    static Eigen::Vector3d difference;
    difference = mLastOdom.position - odom_data.position;
    if (difference.squaredNorm() < 0.01)
        return; // point too close, spline will explode

    mLastOdom = odom_data;

    if (mPathMode == PATH_MODE::BILLBOARDS)
        publishPath();
    else
        write_last_odom();
}

void PathLogger::write_header()
{
    switch (mPathMode) {
        case PATH_MODE::TRAJECTORY: // TUM file format except in csv form
        {
            mTrjFile << std::setprecision(20)
                      << "timestamp,pose.x,pose.y,pose.z,orientation.x,orientation.y,orientation.z,"
                         "orientation.w"
                      << std::endl;
            break;
        }
        case PATH_MODE::BILLBOARDS:
            break;
        default: { // adx plan format
            mTrjFile << std::setprecision(20) << "pose.x,pose.y,pose.z,linear.x,linear.y,linear.z"
                      << std::endl;
            break;
        }
    }
}

void PathLogger::write_last_odom()
{
    switch (mPathMode) {
        case PATH_MODE::TRAJECTORY: {
            mTrjFile << mLastOdom.timestamp << "," << mLastOdom.position.x() << ","
                      << mLastOdom.position.y() << "," << mLastOdom.position.z() << ","
                      << mLastOdom.orientation.x() << "," << mLastOdom.orientation.y() << ","
                      << mLastOdom.orientation.z() << "," << mLastOdom.orientation.w()
                      << std::endl;
            break;
        }
        case PATH_MODE::BILLBOARDS:
            break;
        default: { // adx plan format
            mTrjFile << mLastOdom.position.x() << "," << mLastOdom.position.y() << ","
                      << mLastOdom.position.z() << "," << mLastOdom.linear.x() << ","
                      << mLastOdom.linear.y() << "," << mLastOdom.linear.z() << std::endl;
            break;
        }
    }
}
