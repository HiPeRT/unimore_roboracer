#include "purepursuit_node.hpp"

#include <adx_curve/spline.hpp>

PurePursuitNode::PurePursuitNode()
{
    mNode = std::make_shared<rclcpp::Node>("purepursuit");

    // Declare parameters
    mNode->declare_parameter("params_file", "");
    mNode->declare_parameter("fixed_frame", "map");
    std::string config_file = mNode->get_parameter("params_file").as_string();
    loadConfig(config_file);
    mFixedFrame = mNode->get_parameter("fixed_frame").as_string();

    // Callback groups are what the executor looks for when trying to run multiple threads
    mPoseCallbackGroup =
      mNode->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    mPathToFollowCallbackGroup =
      mNode->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    mGlobalPlanCallbackGroup =
      mNode->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    mLocalPathCallbackGroup =
      mNode->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Everything assigned to a callback group gets bundled into the same thread
    auto pose_sub_opt = rclcpp::SubscriptionOptions();
    pose_sub_opt.callback_group = mPoseCallbackGroup;
    auto path_to_follow_sub_opt = rclcpp::SubscriptionOptions();
    path_to_follow_sub_opt.callback_group = mPathToFollowCallbackGroup;
    auto global_sub_opt = rclcpp::SubscriptionOptions();
    global_sub_opt.callback_group = mGlobalPlanCallbackGroup;
    auto local_sub_opt = rclcpp::SubscriptionOptions();
    local_sub_opt.callback_group = mLocalPathCallbackGroup;

    // Init subscribers
    mPoseSub = mNode->create_subscription<nav_msgs::msg::Odometry>(
      "/odom",
      1,
      std::bind(&PurePursuitNode::poseCallback, this, std::placeholders::_1),
      pose_sub_opt);
    mSuggestedPathSub = mNode->create_subscription<std_msgs::msg::Int8>(
      "/path_to_follow",
      1,
      std::bind(&PurePursuitNode::pathToFollowCallback, this, std::placeholders::_1),
      path_to_follow_sub_opt);
    mGlobalPlanSub = mNode->create_subscription<adx_msgs::msg::Plan>(
      "/plan",
      1,
      std::bind(&PurePursuitNode::globalPlanCallback, this, std::placeholders::_1),
      global_sub_opt);
    mLocalPathSub = mNode->create_subscription<nav_msgs::msg::Path>(
      "/local_path",
      1,
      std::bind(&PurePursuitNode::localPathCallback, this, std::placeholders::_1),
      local_sub_opt);

    // Init publishers
    mDrivePub = mNode->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      "/drive_parameters", 1);
    mArbitratedDrivePub =
      mNode->create_publisher<adx_msgs::msg::ArbitratedAckermannDrive>("/drive_mux", 1);
    mSplinePosePub =
      mNode->create_publisher<geometry_msgs::msg::PoseStamped>("/pp/spline_pos", 10);
    mGoalPub = mNode->create_publisher<geometry_msgs::msg::PoseStamped>("/pp/goal", 10);

    mExecutor.add_node(mNode);

    std::cout << "Ready." << std::endl;
    mExecutor.spin();
}

void PurePursuitNode::pathToFollowCallback(std::shared_ptr<std_msgs::msg::Int8> data)
{
    mSuggestedPath = static_cast<path_type_t>(data->data);
}

void PurePursuitNode::poseCallback(std::shared_ptr<nav_msgs::msg::Odometry> odom)
{
    mPosition.x() = odom->pose.pose.position.x;
    mPosition.y() = odom->pose.pose.position.y;
    mPosition.z() = odom->pose.pose.position.z;

    mOrientation.x() = odom->pose.pose.orientation.x;
    mOrientation.y() = odom->pose.pose.orientation.y;
    mOrientation.z() = odom->pose.pose.orientation.z;
    mOrientation.w() = odom->pose.pose.orientation.w;

    mCurrentSpeed = std::hypot(odom->twist.twist.linear.x, odom->twist.twist.linear.y);

    int ret = controlLoop();

    switch (ret) {
        case 0:
            publishDriveParams();
            publishDebugMarkers();
            break;
        case -2:
            // std::cout << "Current path is null." << std::endl;
            break;
        default:
            std::cout << "WARNING: Unexpected return value from control_loop" << std::endl;
            break;
    }
}

void PurePursuitNode::globalPlanCallback(std::shared_ptr<adx_msgs::msg::Plan> plan)
{
    if (plan->header.frame_id != mFixedFrame)
        std::cout << "WARNING: input plan fixed frame "
                  << "is different from the current purepursuit frame." << std::endl;

    // Build new spline and update it
    adx::data::Plan adx_plan;
    adx::data::fromRos(*plan, adx_plan);
    adx::curve::Spline2D* new_plan = new adx::curve::Spline2D(adx_plan);
    setGlobalPath(new_plan);
    std::cout << "Global plan updated." << std::endl;
}

void PurePursuitNode::localPathCallback(std::shared_ptr<nav_msgs::msg::Path> path)
{
    adx::data::Path adx_path;
    adx::data::fromRos(*path, adx_path);
    adx::curve::Spline2D* new_path = new adx::curve::Spline2D(adx_path);
    setLocalPath(new_path);
}

void PurePursuitNode::publishDriveParams()
{
    // standard message
    ackermann_msgs::msg::AckermannDriveStamped msg;
    msg.drive.speed = mTargetSpeed;
    msg.drive.steering_angle = mTargetSteer;
    mDrivePub->publish(msg);

    // arbitrator message
    adx_msgs::msg::ArbitratedAckermannDrive arb_msg;
    arb_msg.id.data = arb_msg.PP;
    arb_msg.active.data = true;
    arb_msg.drive_parameters.drive.speed = mTargetSpeed;
    arb_msg.drive_parameters.drive.steering_angle = mTargetSteer;
    mArbitratedDrivePub->publish(arb_msg);
}

void PurePursuitNode::publishDebugMarkers()
{
    // Publish current path goal
    geometry_msgs::msg::PoseStamped goal_msg;
    goal_msg.header.frame_id = mFixedFrame;
    goal_msg.pose.position.x = mTargetPose.x();
    goal_msg.pose.position.y = mTargetPose.y();
    mGoalPub->publish(goal_msg);

    // Publish current path position
    geometry_msgs::msg::PoseStamped pos_msg;
    pos_msg.header.frame_id = mFixedFrame;
    pos_msg.pose.position.x = mGlobSplinePose.x();
    pos_msg.pose.position.y = mGlobSplinePose.y();
    mSplinePosePub->publish(pos_msg);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    PurePursuitNode pp;
    rclcpp::shutdown();
    return 0;
}