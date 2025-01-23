#ifndef PUREPURSUIT_HPP
#define PUREPURSUIT_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <adx_curve/curve.hpp>
#include <chrono>

#define _USE_MATH_DEFINES
#include <cmath>

#include <mutex>
#include <string>

typedef enum path_type_t { PATH_NONE = -1, PATH_GLOBAL = 0, PATH_FRENET = 1 } path_type_t;
typedef enum launch_mode_t { LM_SINGLEPATH = 0, LM_FRENET = 1 } launch_mode_t;

/**
 * Transforms a point from the global coordinate system to local coordinates
 * @brief Point transform
 * @param aPoint the point to transform
 * @param aPose  the current pose in global coordinates, aka the transform vector
 * @param aYaw   the transform yaw angle
 * @return Eigen::Vector3d the transformed point
 */
static inline Eigen::Vector3d pointTransform(const Eigen::Vector3f aPoint, const Eigen::Vector3f aPose,
                                             const float aYaw) {
    Eigen::Vector3d local_straight;
    local_straight.x() = aPoint.x() - aPose.x();
    local_straight.y() = aPoint.y() - aPose.y();

    Eigen::Vector3d ret;
    ret.x() = local_straight.x() * cos(aYaw) + local_straight.y() * sin(aYaw);
    ret.y() = -local_straight.x() * sin(aYaw) + local_straight.y() * cos(aYaw);

    return ret;
}

/**
 * This class implements the logic to control a vehicle via the pure pursuit method.
 * @brief Pure Pursuit
 */
class PurePursuit {
   public:
    /**
     * @brief Construct a new Pure Pursuit object
     */
    PurePursuit();

    /**
     * Applies the pure pursuit method to the most recent data
     * @brief Main control loop
     * @returns 0 on sucess or -2 if the selected path is null
     */
    int controlLoop();

    /**
     * Set the suggested path to follos
     * @brief Set the desired path to follow
     *
     * -1 -> none
     *  0 -> global
     *  1 -> local
     *
     * @param aPathToFollow the path to follow code
     */
    void setSuggestedPath(path_type_t aPathToFollow);

    /**
     * Sets a new global plan, discarding the old one.
     * @brief Set a new Global Path
     *
     * @param aGlobalPath The new global path to follow
     */
    void setGlobalPath(Curve* aGlobalPath);

    /**
     * Sets a new local path, discarding the old one.
     * @brief Set a new Local Path
     *
     * @param aLocalPath The new local path to follow
     */
    void setLocalPath(Curve* aLocalPath);

    /**
     * Returns the current spline pose.
     * @brief Get spline pose
     *
     * @param aPose the output spline pose
     */
    void getSplinePose(Eigen::Vector3d& aPose);

    /**
     * Returns the current target pose.
     * @brief Get target pose
     *
     * @param aPose the output target pose
     */
    void getTargetPose(Eigen::Vector3d& aPose);

   protected:
#ifdef __linux__
    void loadConfig(std::string aYamlConfig);
#else  // Erika
    void loadConfig();
#endif

    /**
     * Returns the steering angle needed to reach the desired path goal
     * @brief Optimal purepursuit steering
     * @param aTarget The target point in local coordinates (base link)
     * @param aSpeed The current speed of the vehicle
     * @return The optimal steering
     */
    float optimalSteering(Eigen::Vector3f aTarget, float aSpeed);

    /**
     * locks used to access the global plan and local path data structures
     */
    std::mutex mGlobalPlanLock, mLocalPathLock;

    /**
     * Current position in global coordinates
     */
    Eigen::Vector3f mPosition;

    /**
     * Current orientation in global coordinates
     */
    Eigen::Quaternionf mOrientation;

    /**
     * Current longitudinal speed
     */
    float mCurrentSpeed;
    /**
     * Current spline distance
     */
    float mCurrentDistance;

    /**
     * Drive parameters, target steering angle and target speed
     */
    float mTargetSteer, mTargetSpeed;

    /**
     * Vehicle related parameters
     */
    float mWheelBase, mMinSteer, mMaxSteer, mMaxSpeed, mMinSpeed, mAccelMax;

    /**
     * Steering smoothing gain based on speed
     */
    float mLookForwardGain;

    /**
     * Global plan to follow
     */
    Curve* mGlobalPlan = nullptr;

    /**
     * Local plan to follow
     */
    Curve* mLocalPath = nullptr;

    /**
     * Suggested path/plan to follow
     */
    path_type_t mSuggestedPath;

    /**
     * Current launch mode
     */
    launch_mode_t mLaunchMode;

    /**
     * Debug points on spline
     */
    Eigen::Vector3f mGlobSplinePose, mTargetPose;

    /**
     * Current configuration parameters
     */
    float mLookAhead, mSpeedScaleFactor;
};

#endif  // PUREPURSUIT_HPP
