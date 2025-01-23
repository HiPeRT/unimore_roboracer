#include <adx_curve/segment.hpp>

#include "purepursuit_node.hpp"

PurePursuitNode::PurePursuitNode() {
    sub_node_ = std::make_shared<rclcpp::Node>("purepursuit_sub_node");
    pub_node_ = std::make_shared<rclcpp::Node>("purepursuit_pub_node");

    // Declare parameters
    pub_node_->declare_parameter("params_file");
    std::string config_file = pub_node_->get_parameter("params_file").as_string();
    loadConfig(config_file);

    pub_node_->declare_parameter("fixed_frame");
    fixed_frame = pub_node_->get_parameter("fixed_frame").as_string();

    // Callback groups are what the executor looks for when trying to run multiple threads
    callback_group_pose_sub_ = sub_node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_path_to_follow_sub_ = sub_node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_plan_sub_ = sub_node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_frenet_sub_ = sub_node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Everything assigned to a callback group gets bundled into the same thread
    auto pose_sub_opt = rclcpp::SubscriptionOptions();
    pose_sub_opt.callback_group = callback_group_pose_sub_;
    auto path_to_follow_sub_opt = rclcpp::SubscriptionOptions();
    path_to_follow_sub_opt.callback_group = callback_group_path_to_follow_sub_;
    auto plan_sub_opt = rclcpp::SubscriptionOptions();
    plan_sub_opt.callback_group = callback_group_plan_sub_;
    auto frenet_sub_opt = rclcpp::SubscriptionOptions();
    frenet_sub_opt.callback_group = callback_group_frenet_sub_;

    // Init subscribers
    pose_sub = sub_node_->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 1, std::bind(&PurePursuitNode::poseCallback, this, std::placeholders::_1), pose_sub_opt);
    path_to_follow_sub = sub_node_->create_subscription<std_msgs::msg::Int8>(
        "/path_to_follow", 1, std::bind(&PurePursuitNode::pathToFollowCallback, this, std::placeholders::_1),
        path_to_follow_sub_opt);
    plan_sub = sub_node_->create_subscription<adx_msgs::msg::Plan>(
        "/plan", 1, std::bind(&PurePursuitNode::globalPlanCallback, this, std::placeholders::_1), plan_sub_opt);
    frenet_sub = sub_node_->create_subscription<nav_msgs::msg::Path>(
        "/local_path", 1, std::bind(&PurePursuitNode::localPathCallback, this, std::placeholders::_1), frenet_sub_opt);

    // Init publishers
    drive_pub = pub_node_->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive_parameters", 1);
    spline_pos_pub = pub_node_->create_publisher<geometry_msgs::msg::PoseStamped>("/pp/spline_pos", 10);
    goal_pub = pub_node_->create_publisher<geometry_msgs::msg::PoseStamped>("/pp/goal", 10);

    executor.add_node(pub_node_);
    executor.add_node(sub_node_);

    std::cout << "Ready." << std::endl;
    executor.spin();
}

/**
 * @brief Path to follow callback
 */
void PurePursuitNode::pathToFollowCallback(const std_msgs::msg::Int8::SharedPtr data) {
    suggested_path_ = static_cast<path_type_t>(data->data);
}

/**
 * @brief Odometry callback
 */
void PurePursuitNode::poseCallback(const nav_msgs::msg::Odometry::SharedPtr odom) {
    position_.x() = odom->pose.pose.position.x;
    position_.y() = odom->pose.pose.position.y;
    position_.z() = odom->pose.pose.position.z;

    orientation_.x() = odom->pose.pose.orientation.x;
    orientation_.y() = odom->pose.pose.orientation.y;
    orientation_.z() = odom->pose.pose.orientation.z;
    orientation_.w() = odom->pose.pose.orientation.w;

    current_speed_ = odom->twist.twist.linear.x;

    local_path_lock_.lock();
    int ret = controlLoop();
    local_path_lock_.unlock();

    switch (ret) {
        case 0:
            publishDriveParams();
            // fall through
        case -1:
            publishDebugMarkers();
            break;
        case -2:
            std::cout << "No position received yet" << std::endl;
            break;
        case -3:
            std::cout << "Current path is null." << std::endl;
            break;
        default:
            std::cout << "WARNING: Unexpected return value from control_loop" << std::endl;
            break;
    }
}

/**
 * @brief Global plan callback
 */
void PurePursuitNode::globalPlanCallback(const adx_msgs::msg::Plan::SharedPtr plan) {
    Vec_d xs, ys, speeds;

    if (plan->header.frame_id != fixed_frame)
        std::cout << "WARNING: input plan fixed frame "
                  << "is different from the current purepursuit map frame." << std::endl;

    for (adx_msgs::msg::PlanPoint point : plan->points) {
        xs.push_back(point.position.x);
        ys.push_back(point.position.y);
        speeds.push_back(hypot(point.speed.x, point.speed.y));
    }

    // Calculate new segment and update it
    Segment* new_plan = new Segment(&xs, &ys, &speeds);
    global_plan_lock_.lock();
    setGlobalPath(new_plan);
    global_plan_lock_.unlock();
    std::cout << "Global plan updated." << std::endl;
}

void PurePursuitNode::localPathCallback(const nav_msgs::msg::Path::SharedPtr path) {
    Vec_d xs, ys;

    for (geometry_msgs::msg::PoseStamped pose : path->poses) {
        xs.push_back(pose.pose.position.x);
        ys.push_back(pose.pose.position.y);
    }

    Segment* new_path = new Segment(&xs, &ys);
    local_path_lock_.lock();
    setLocalPath(new_path);
    local_path_lock_.unlock();
}

void PurePursuitNode::publishDriveParams() {
    ackermann_msgs::msg::AckermannDriveStamped msg;
    msg.drive.speed = target_speed_;
    msg.drive.steering_angle = -target_steer_;
    drive_pub->publish(msg);
}

void PurePursuitNode::publishDebugMarkers() {
    // Publish current goal on path
    geometry_msgs::msg::PoseStamped goal_msg;
    goal_msg.header.frame_id = fixed_frame;
    goal_msg.pose.position.x = target_pose_.x();
    goal_msg.pose.position.y = target_pose_.y();
    goal_pub->publish(goal_msg);

    // Publish current position on the path segment
    geometry_msgs::msg::PoseStamped pos_msg;
    pos_msg.header.frame_id = fixed_frame;
    pos_msg.pose.position.x = glob_s_pose_.x();
    pos_msg.pose.position.y = glob_s_pose_.y();
    spline_pos_pub->publish(pos_msg);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    PurePursuitNode pp;
    rclcpp::shutdown();
    return 0;
}
