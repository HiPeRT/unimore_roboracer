#ifndef PARTICLE_FILTER_PARTICLE_FILTER_NODE_HPP
#define PARTICLE_FILTER_PARTICLE_FILTER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <adx_data_ros/adx_data_ros.hpp>


#include "particle_filter/particle_filter.hpp"
#include "ray_casting/ray_marching.hpp"


#define ANGLE_STEP (0.00435928675336234f)

/**
 * @brief Particle Filter Node class
 * This class represents the ROS2 wrapper of the Particle Filter
 * @tparam ParticleFilterT The particle filter implementation
 */
template<class ParticleFilterT>
class ParticleFilterNode : public ParticleFilterT
{
  public:
    using ParticleFilterT::mMapDescriptor;
    using ParticleFilterT::mParticles;
    using ParticleFilterT::mPose;
    using ParticleFilterT::ParticleFilterT;

    /**
     * @brief Construct a new Particle Filter Node
     *
     * @param aSensorModel The sensor model implementation to be used for weighing the particles
     */
    ParticleFilterNode(IRayCasting& aSensorModel)
      : ParticleFilterT(aSensorModel)
    {
        mNode = std::make_shared<rclcpp::Node>("particle_node");
        mTfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(mNode);

        mNode->declare_parameter("conf_path");
        std::string conf_path = mNode->get_parameter("conf_path").as_string();

        mNode->declare_parameter("numRaysDs");
        numRaysDs = mNode->get_parameter("numRaysDs").as_int();

        // temporary, will move to constructor
        ParticleFilterT::setConfig(conf_path);
        std::cout << "[INFO] Configuration loaded." << std::endl;

        // Subscribers
        mMapSub = mNode->create_subscription<nav_msgs::msg::OccupancyGrid>(
          "/map", 1, std::bind(&ParticleFilterNode::mapCallback, this, std::placeholders::_1));

        mClickSub = mNode->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/initialpose",
          1,
          std::bind(&ParticleFilterNode::initialPoseCallback, this, std::placeholders::_1));

        mOdomSub = mNode->create_subscription<nav_msgs::msg::Odometry>(
          "/odom",
          1,
          std::bind(&ParticleFilterNode::odometryCallback, this, std::placeholders::_1));

        mLidarSub = mNode->create_subscription<sensor_msgs::msg::LaserScan>(
          "/scan", 1, std::bind(&ParticleFilterNode::cloudCallback, this, std::placeholders::_1));

        // Publishers
        mPosePub =
          mNode->create_publisher<nav_msgs::msg::Odometry>("/pf/pose", 1);
        mParticlesPub = mNode->create_publisher<geometry_msgs::msg::PoseArray>(
          "/pf/particles", 1);
        mScanPub =
          mNode->create_publisher<sensor_msgs::msg::LaserScan>("/pf/scan", 1);
    }

    /**
     * @brief Start running
     * Spins the ROS2 node.
     */
    void run() { rclcpp::spin(mNode); }

  protected:
    // ros
    rclcpp::Node::SharedPtr mNode;
    std::shared_ptr<tf2_ros::TransformBroadcaster> mTfBroadcaster;

    // subs
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mMapSub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr mClickSub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mOdomSub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr mLidarSub;

    // pubs
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr mPosePub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr mParticlesPub;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr mScanPub;

    // particle filter node
    bool mPoseReceived = false;
    bool mMapInitialized = false;

    // first odometry received
    bool mFirstOdom = true;

    // Latest lidar packet timestamp
    adx::data::TimeStamp mLatestLidarTs;

    // Latest odometry
    adx::data::Odometry mLatestOdom;

    // Latest twist data
    adx::data::Twistd mLatestTwist;

    // downsample
    int numRaysDs;

    // callbacks

    /**
     * @brief Map callback
     * @param aRosMap the map to localize into
     */
    void mapCallback(const std::shared_ptr<nav_msgs::msg::OccupancyGrid> aRosMap)
    {
        std::cout << "[INFO] Processing map." << std::endl;

        // extract map info
        mMapDescriptor.map_height = aRosMap->info.height;
        mMapDescriptor.map_width = aRosMap->info.width;
        mMapDescriptor.map_originX = aRosMap->info.origin.position.x;
        mMapDescriptor.map_originY = aRosMap->info.origin.position.y;
        mMapDescriptor.map_resolution = aRosMap->info.resolution;
        mMapDescriptor.MAX_RANGE_PX =
          ParticleFilterT::mCloudDescriptor.maxRangeMeters / mMapDescriptor.map_resolution;
        mMapDescriptor.opp_originX =
          (mMapDescriptor.map_width * mMapDescriptor.map_resolution) + mMapDescriptor.map_originX;
        mMapDescriptor.opp_originY = -mMapDescriptor.map_originY;

        // permissible region
        Eigen::MatrixXi array_255 =
          Eigen::MatrixXi(mMapDescriptor.map_height, mMapDescriptor.map_width);
        for (int i = 0; i < mMapDescriptor.map_width * mMapDescriptor.map_height; i++) {
            int r = i / mMapDescriptor.map_width;
            int c = i % mMapDescriptor.map_width;
            array_255(r, c) = aRosMap->data[i];
        }
        ParticleFilterT::preprocessMap(array_255);
        std::cout << "[INFO] Map processed." << std::endl;
        mMapInitialized = true;
    }

    /**
     * @brief Initial pose callback
     * @param aRosPose the starting position of the vehicle
     */
    void initialPoseCallback(
      const std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped> aRosPose)
    {
        adx::data::Pose2f initial_pose;
        adx::data::fromRos(aRosPose->pose.pose, initial_pose);
        ParticleFilterT::spawnParticles(initial_pose);
        publishParticles();
        mPoseReceived = true;
    }

    /**
     * @brief Odometry callback
     * @param aRosOdom the vehicle odometry
     */
    void odometryCallback(const std::shared_ptr<nav_msgs::msg::Odometry> aRosOdom)
    {
        adx::data::Odometry odom;
        fromRos(*aRosOdom, odom);

        // check if this is the first odometry
        if (mFirstOdom) {
            mLatestOdom = odom;
            mFirstOdom = false;
            return;
        }

        // Find latest twist data
        float dt_sec = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         odom.timestamp.timestamp - mLatestOdom.timestamp.timestamp)
                         .count() *
                       1e-9f;

        // avoid crashing on negative timestamps
        if (dt_sec < 0.0f)
            dt_sec = 0.01f;

        mLatestTwist.linear = odom.linear;
        mLatestTwist.angular = odom.angular;
        // yaw_rate = (from.inverse() * to).yaw() / dt
        // mLatestTwist.angular.z() = (mLatestOdom.orientation.inverse() * odom.orientation)
        //                               .toRotationMatrix()
        //                               .eulerAngles(0, 1, 2)[2] /
        //                             dt_sec;

        // Replace latest odometry
        mLatestOdom = odom;
    }

    /**
     * @brief Cloud callback
     * @param aRosScan A 2D lidar pointcloud
     */
    void cloudCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> aRosScan)
    {
        if (!mMapInitialized || !mPoseReceived) {
            fromRos(aRosScan->header.stamp, mLatestLidarTs);
            return;
        }

        std::vector<float> ds_ranges;
        std::vector<float> rays_angles;
        ds_ranges.resize(numRaysDs);
        rays_angles.resize(numRaysDs);

        // find number of valid rays
        int numValidRays = 0;
        for (size_t i = 0; i < aRosScan->ranges.size(); i++) {
            if (aRosScan->ranges[i] < aRosScan->range_max &&
                aRosScan->ranges[i] > aRosScan->range_min)
                numValidRays++;
        }

        // downsample - nearest neighbor without multiplications or divisions
        int j = 0;
        int tmp = numValidRays;
        for (size_t i = 0; i < aRosScan->ranges.size(); ++i) {
            tmp += numRaysDs - 1;
            if (tmp >= numValidRays) {
                tmp -= numValidRays;
                while (i < aRosScan->ranges.size() &&
                       (aRosScan->ranges[i] > aRosScan->range_max ||
                        aRosScan->ranges[i] < aRosScan->range_min)) {
                    aRosScan->ranges[i] = 0.0f;
                    i++;
                }

                rays_angles[j] = i * ANGLE_STEP;
                ds_ranges[j] = aRosScan->ranges[i];
                j++;
                if (j >= numRaysDs)
                    break;
            } else {
                aRosScan->ranges[i] = 0.0f;
            }
        }

        while (j < numRaysDs) {
            ds_ranges[j] = 0.0f;
            j++;
        }

        adx::data::TimeStamp lidar_ts;
        fromRos(aRosScan->header.stamp, lidar_ts);
        double dt = std::chrono::duration_cast<std::chrono::nanoseconds>(lidar_ts.timestamp -
                                                                         mLatestLidarTs.timestamp)
                      .count() *
                    1e-9;
        mLatestLidarTs = lidar_ts;

        // update
        ParticleFilterT::updateLocalization(mLatestTwist, dt, ds_ranges, rays_angles);

        // publish downsampled scan
        sensor_msgs::msg::LaserScan downsampled_msg = *aRosScan;
        mScanPub->publish(downsampled_msg);
    }

    void publishEstimate() override
    {
        // TODO: add parameters to turn on/off publishing
        publishPose();
        publishTransform();
        publishParticles();
    }

    /**
     * @brief Publish the estimated pose
     *
     * Publish an Odometry message containing the estimated pose
     */
    void publishPose()
    {
        // TODO: create assignment into adx::data::Odometry to parse adx::Pose2X
        adx::data::Odometry odom;
        odom.timestamp = mLatestLidarTs;
        odom.frame_id = "map";
        // copy position
        odom.position.x() = mPose.position.x();
        odom.position.y() = mPose.position.y();
	      odom.position.z() = 0.0f;
        // build eigen quaternion
        odom.orientation = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
                           Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(mPose.yaw(), Eigen::Vector3d::UnitZ());
        // copy twist
        odom.linear = mLatestTwist.linear;

        // copy position covariance
        odom.pose_covariance(0, 0) = mPose.pose_covariance(0, 0);
        odom.pose_covariance(1, 1) = mPose.pose_covariance(1, 1);
        odom.pose_covariance(2, 2) = 0.0f;

        // copy rotation covariance
        odom.pose_covariance(3, 3) = 0.0f;
        odom.pose_covariance(4, 4) = 0.0f;
        odom.pose_covariance(5, 5) = mPose.pose_covariance(2, 2);

        nav_msgs::msg::Odometry rosOdom;
        adx::data::toRos(odom, rosOdom);

        mPosePub->publish(rosOdom);
    }

    /**
     * @brief Publish the current particle states
     *
     * Publish a PoseArray message containing the particle poses
     */
    void publishParticles()
    {
        int maxParticles = std::min(static_cast<int>(mParticles.size()), 100);
        geometry_msgs::msg::PoseArray particles_msg;
        particles_msg.header.frame_id = "map";
        builtin_interfaces::msg::Time rosTimestamp;
        adx::data::toRos(mLatestLidarTs, rosTimestamp);
        particles_msg.header.stamp = rosTimestamp;
        particles_msg.poses.resize(maxParticles);

        for (int i = 0; i < maxParticles; i++) {
            particles_msg.poses[i].position.x = mParticles[i].position.x();
            particles_msg.poses[i].position.y = mParticles[i].position.y();
            particles_msg.poses[i].position.z = 0.0f;

            tf2::Quaternion q;
            q.setRPY(0, 0, mParticles[i].yaw());
            particles_msg.poses[i].orientation.x = q.x();
            particles_msg.poses[i].orientation.y = q.y();
            particles_msg.poses[i].orientation.z = q.z();
            particles_msg.poses[i].orientation.w = q.w();
        }
        mParticlesPub->publish(particles_msg);
    }

    /**
     * @brief Publish the lidar_link to map transform
     *
     * Publish a TransformStamped containing the transformation from lidar_link to map using the
     * estimated pose
     */
    void publishTransform()
    {
        geometry_msgs::msg::TransformStamped ts;

        builtin_interfaces::msg::Time rosTimestamp;
        adx::data::toRos(mLatestLidarTs, rosTimestamp);
        ts.header.stamp = rosTimestamp;
        // map to lidar breaks tf tree
        ts.header.frame_id = "lidar_link";
        ts.child_frame_id = "map";

        ts.transform.translation.x = mPose.position.x();
        ts.transform.translation.y = mPose.position.y();
        ts.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, mPose.yaw());
        ts.transform.rotation.x = q.x();
        ts.transform.rotation.y = q.y();
        ts.transform.rotation.z = q.z();
        ts.transform.rotation.w = q.w();

        tf2::Transform tf;
        tf2::fromMsg(ts.transform, tf);
        ts.transform = tf2::toMsg(tf.inverse());

        mTfBroadcaster->sendTransform(ts);
    }
};

#endif // PARTICLE_FILTER_PARTICLE_FILTER_NODE_HPP
