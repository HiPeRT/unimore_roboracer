#ifdef __linux__
#include <yaml-cpp/yaml.h>
#else // Erika
#include <inmate.h>
#endif

#include "purepursuit/purepursuit.hpp"

#include <iostream>

PurePursuit::PurePursuit() {}

#ifdef __linux__
void PurePursuit::loadConfig(std::string aYamlConfig)
{
    // Init parameters from yaml configuration
    YAML::Node params = YAML::LoadFile(aYamlConfig);

    mSpeedScaleFactor = params["speed_scale_factor"].as<float>();

    // vehicle parameters
    mMaxSpeed = params["max_speed"].as<float>();
    mMinSpeed = params["min_speed"].as<float>();
    mMaxSteer = params["max_steer"].as<float>();
    mMinSteer = params["min_steer"].as<float>();

    // lookahead
    mLookAhead = params["look_ahead"].as<float>();

    // Vehicle parameters
    mWheelBase = params["wheel_base"].as<float>();
    mLookForwardGain = params["look_forward_gain"].as<float>();
    mAccelMax = params["accel_max"].as<float>();

    mLaunchMode = static_cast<launch_mode_t>(params["launch_mode"].as<int>());

    mGlobalPlan = nullptr;
    mLocalPath = nullptr;
    mSuggestedPath = PATH_GLOBAL;
}

#else // Erika
void PurePursuit::loadConfig()
{
    printk("purepursuit!\n");

    char buf[20];

    max_speed = atof(cmdline_parse_str("max_speed", buf, sizeof(buf), "1.0"));
    min_speed = atof(cmdline_parse_str("min_speed", buf, sizeof(buf), "1.0"));
    speed_scale_factor = atof(cmdline_parse_str("speed_scale_factor", buf, sizeof(buf), "1.0"));
    printk("max_speed: %d / 100\n", (int)(max_speed * 100));
    printk("min_speed: %d / 100\n", (int)(min_speed * 100));
    printk("speed_scale_factor: %d / 100\n", (int)(speed_scale_factor * 100));

    // steer parameters
    max_steer = atof(cmdline_parse_str("max_steer", buf, sizeof(buf), "0.523"));
    min_steer = atof(cmdline_parse_str("min_steer", buf, sizeof(buf), "-0.523"));
    printk("max_steer: %d / 100\n", (int)(max_steer * 100));
    printk("min_steer: %d / 100\n", (int)(min_steer * 100));

    // lookahead parameters
    mLookAhead = atof(cmdline_parse_str("look_ahead", buf, sizeof(buf), "10.0"));;
    printk("look_ahead: %d / 100\n", (int)(look_ahead * 100));

    // Vehicle parameters
    wheel_base = atof(cmdline_parse_str("wheel_base", buf, sizeof(buf), "3.171"));
    look_forward_gain = atof(cmdline_parse_str("look_forward_gain", buf, sizeof(buf), "0.9"));
    accel_max = atof(cmdline_parse_str("accel_max", buf, sizeof(buf), "4.0"));
    printk("wheel_base: %d / 100\n", (int)(wheel_base * 100));
    printk("look_forward_gain: %d / 100\n", (int)(look_forward_gain * 100));
    printk("accel_max: %d / 100\n", (int)(accel_max * 100));

    launch_mode = static_cast<launch_mode_t>(cmdline_parse_int("launch_mode", 0));
    printk("launch_mode %d\n", (int)launch_mode);

    global_plan_ = nullptr;
    local_path_ = nullptr;
    suggested_path = PATH_GLOBAL;
}
#endif

float PurePursuit::optimalSteering(Eigen::Vector3f aTarget, float aSpeed)
{
    float alpha = atan2(aTarget.y(), aTarget.x() - mWheelBase); // (y, x-L)
    alpha = std::max(-M_PI_2f32, std::min(alpha, M_PI_2f32));
    float Lf = mLookForwardGain * aSpeed + mLookAhead;
    float delta = atan2(2.0 * mWheelBase * sin(alpha) / Lf, 1.0);
    delta = std::max(mMinSteer, std::min(delta, mMaxSteer));
    return delta;
}

int PurePursuit::controlLoop()
{
    // std::chrono::time_point<std::chrono::steady_clock> time_tmp =
    // std::chrono::steady_clock::now(); control_delta_time = time_tmp - control_last_timestamp;
    // control_last_timestamp = time_tmp;
    Curve* cur_path;
    int ret = 0;

    mLocalPathLock.lock();
    mGlobalPlanLock.lock();

    // Choose path to follow
    switch (mSuggestedPath) {
        case PATH_GLOBAL:
            cur_path = mGlobalPlan;
            break;

        case PATH_FRENET:
            cur_path = mLocalPath;
            break;

        default:
            cur_path = nullptr;
            break;
    }

    // Update position on all paths if they exist
    if (mGlobalPlan != nullptr)
        mGlobalPlan->getCurveDistance(mPosition, mCurrentDistance);
    if (mLocalPath != nullptr)
        mLocalPath->getCurveDistance(mPosition, mCurrentDistance);

    if (cur_path == nullptr) {
        // Brake to zero if there's no feasible path or if path is nullptr
        // drive_parameters.target_speed -= accel_max * (1.0 / control_delta_time.count());
        // drive_parameters.target_speed = std::max(drive_parameters.target_speed, 0.0);
        // drive_parameters.target_steer = 0.0;
        ret = -2;
    } else {
        // Get current position on the spline
        cur_path->getPosition(mCurrentDistance, mGlobSplinePose);
        // PP optimal speed
        mGlobalPlan->getSpeed(mCurrentDistance, mTargetSpeed);
        mTargetSpeed *= mSpeedScaleFactor;
        mTargetSpeed = std::min(mMaxSpeed, std::max(mMinSpeed, mTargetSpeed));

        // PP optimal steer
        cur_path->getPosition(mCurrentDistance + mLookAhead, mTargetPose);

        Eigen::Isometry3f transform;
        mOrientation.normalize();
        transform.linear() = mOrientation.toRotationMatrix();
        transform.translation() = mPosition;

        Eigen::Vector3f target_pose_local = transform.inverse() * mTargetPose;

        mTargetSteer = optimalSteering(target_pose_local, mTargetSpeed);
    }

    mLocalPathLock.unlock();
    mGlobalPlanLock.unlock();
    return ret;
}

void PurePursuit::setGlobalPath(Curve* aGlobalPath)
{
    mGlobalPlanLock.lock();
    delete mGlobalPlan;
    mGlobalPlan = aGlobalPath;
    mGlobalPlanLock.unlock();
}

void PurePursuit::setLocalPath(Curve* aLocalPath)
{
    mLocalPathLock.lock();
    delete mLocalPath;
    mLocalPath = aLocalPath;
    mLocalPathLock.unlock();
}