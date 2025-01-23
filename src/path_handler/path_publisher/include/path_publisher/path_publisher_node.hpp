#ifndef PATH_PUBLISHER_PATH_PUBLISHER_NODE_HPP
#define PATH_PUBLISHER_PATH_PUBLISHER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "adx_msgs/msg/plan.hpp"

#include "path_publisher.hpp"

class PathPublisherNode : public PathPublisher
{
  public:
    PathPublisherNode();
    ~PathPublisherNode();

    void publishPlan();
    void publishPath();
    void publishBin();

    void publish();

    rclcpp::Node::SharedPtr mNode;

  private:
    PATH_MODE mPathMode;
    std::string mFixedFrame;
    std::string mBinName;

    // adx_msgs::msg::Plan::SharedPtr plan_msg_;
    rclcpp::Publisher<adx_msgs::msg::Plan>::SharedPtr mPlanPub;

    // nav_msgs::msg::Path::SharedPtr path_msg_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mPathPub;
};

#endif // PATH_PUBLISHER_PATH_PUBLISHER_NODE_HPP
