#include <iostream>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <adx_msgs/msg/plan.hpp>
#include <nav_msgs/msg/path.hpp>
#include <adx_data_ros/adx_data_ros.hpp>

#include "path_publisher/path_publisher_node.hpp"

PathPublisherNode::PathPublisherNode()
{
    mNode = std::make_shared<rclcpp::Node>("path_publisher_node");

    int tmp;
    mNode->declare_parameter("path_mode", 0);
    mNode->declare_parameter("fixed_frame", "map");
    mNode->declare_parameter("filename", "logged_trj.csv");
    mNode->get_parameter<int>("path_mode", tmp);
    mPathMode = static_cast<PATH_MODE>(tmp);
    mNode->get_parameter<std::string>("fixed_frame", mFixedFrame);
    mNode->get_parameter<std::string>("filename", mFilename);

    if (mPathMode & PATH_MODE::PLAN) {
        std::cout << "Creating plan publisher" << std::endl;
        mPlanPub = mNode->create_publisher<adx_msgs::msg::Plan>("/plan", 10);
    }
    if (mPathMode & PATH_MODE::PATH) {
        std::cout << "Creating path publisher" << std::endl;
        mPathPub = mNode->create_publisher<nav_msgs::msg::Path>("/path", 10);
    }
    if (mPathMode & PATH_MODE::BIN) {
        mNode->declare_parameter("binary_file", "logged_trj.bin");
        mNode->get_parameter<std::string>("binary_file", mBinName);
    }

    std::cout << "Loading " << mFilename << std::endl;
    read_plan();
    std::cout << mFilename << " loaded." << std::endl;
}

PathPublisherNode::~PathPublisherNode()
{
    rclcpp::shutdown();
}

void PathPublisherNode::publish()
{
    if (mPathMode & PATH_MODE::PATH) {
        publishPath();
    }
    if (mPathMode & PATH_MODE::PLAN) {
        publishPlan();
    }
    if (mPathMode & PATH_MODE::BIN) {
        publishBin();
    }
    std::cout << "All done." << std::endl;
}

void PathPublisherNode::publishPath()
{
    std::cout << "Publishing path" << std::endl;
    mPlan.timestamp = std::chrono::steady_clock::now();
    mPlan.frame_id = mFixedFrame;
    nav_msgs::msg::Path path;
    adx::data::toRos(mPlan, path);

    mPathPub->publish(path);
}

void PathPublisherNode::publishPlan()
{
    std::cout << "Publishing plan" << std::endl;
    mPlan.timestamp = std::chrono::steady_clock::now();
    mPlan.frame_id = mFixedFrame;
    adx_msgs::msg::Plan plan;
    adx::data::toRos(mPlan, plan);

    mPlanPub->publish(plan);
}

void PathPublisherNode::publishBin()
{
    std::cout << "Writing to " << mBinName << std::endl;

    std::ofstream bin_file;
    bin_file.open(mBinName, std::ios::binary | std::ios::out);
    if (!bin_file.good()) {
        std::cerr << "bad file" << std::endl;
        exit(-1);
    }

    uint64_t tmp = mPlan.positions.size();
    bin_file.write(reinterpret_cast<const char*>(&tmp), sizeof(tmp));

    for (unsigned int i = 0; i < mPlan.positions.size(); ++i) {
        bin_file.write(reinterpret_cast<const char*>(&mPlan.positions[i].position.x()),
                        sizeof(double));
        bin_file.write(reinterpret_cast<const char*>(&mPlan.positions[i].position.y()),
                        sizeof(double));
        bin_file.write(reinterpret_cast<const char*>(&mPlan.speeds[i].x()), sizeof(double));
    }

    bin_file.close();
}