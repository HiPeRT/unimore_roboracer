#ifndef PARTICLE_FILTER_PARTICLE_FILTER_HPP
#define PARTICLE_FILTER_PARTICLE_FILTER_HPP

#include <opencv2/opencv.hpp>
#include <random>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "ray_casting/ray_casting.hpp"

#include <adx_data/odometry.hpp>
#include <adx_data/particle.hpp>
#include <adx_data/pose.hpp>

/**
 * @brief ParticleFilter class
 *
 * This represents the CPU implementation of the Particle Filter.
 */
class ParticleFilter
{
  public:
    /**
     * @brief Constructs a new Particle Filter object
     * @param aConfFile yaml configuration file
     * @param aSensorModel reference to any sensor model method
     */
    ParticleFilter(IRayCasting& aSensorModel);

    /**
     * @brief Set YAML configuration
     * Parses a YAML configuration from the argument path
     *
     * @param aConfFile the path to the yaml file
     */
    void setConfig(std::string aConfFile);

    /**
     * @brief Update pose
     *
     * Updates the estimated pose and covariance given
     * the current particle distribution
     */
    virtual void updatePose();

    /**
     * @brief Preprocess the map
     *
     * Using the provided map it calculates the permissible region and the distance map
     *
     * @param aMap the map to process
     */
    virtual void preprocessMap(Eigen::MatrixXi& aMap);

    /**
     * @brief Spawns the particles around the provided pose
     *
     * Spawns the particles in the close proximity of the provided pose with a uniform distribution.
     *
     * @param aInitialPose initial pose where to spawn the particles
     */
    virtual void spawnParticles(adx::data::Pose2f aInitialPose);

    /**
     * @brief Localization update
     * Implements the localization update step for each particle and the pose given a certain
     * odometry and scan
     *
     * @param aTwist the twist data to use in the motion model
     * @param aDt the time passed since the last iteration
     * @param aDownsampledScan the scan to use in the sensor model
     * @param aRaysAngles the angle for each ray in the downsampled scan

     */
    void updateLocalization(adx::data::Twistd& aTwist,
                            float aDt,
                            std::vector<float>& aDownsampledScan,
                            std::vector<float>& aRaysAngles);

    /**
     * @brief Resample the particles around the latest expected pose
     *
     * Resamples the particles around the latest expected pose
     * @param aNumParticles The maximum number of particles
     */
    virtual void resampleParticles();

    /**
     * @brief Applies the motion model to the particles
     *
     * The motion model applies the odometry to the entire particle distribution. Since there the
     * odometry data is inaccurate, the motion model mixes in gaussian noise to spread out the
     * distribution. Vectorized motion model. Computing the motion model over all particles is
     * thousands of times faster than doing it for each particle individually due to vectorization
     * and reduction in function call overhead
     *
     * @param aVelocity  current velocity form odometry
     * @param aDt delta of time between two lidar scan
     * @param aYawRate difference of yaw between the actual yaw and the previous
     */
    virtual void applyMotionModel(float aVelocity, float aDt, float aYawRate);

    /**
     * @brief Applies the sensor model to each particle
     *
     * Applies the sensor model to each particle to estimate its weight in relation to the measured
     * point cloud provided.
     * @param aCloud the measured lidar point cloud
     * @param aRaysAngles the downsampled rays angles
     */
    virtual void applySensorModel(std::vector<float>& aCloud, std::vector<float>& aRaysAngles);

    /**
     * @brief Normalize the weights
     *
     * Normalize the weights of the particles making the sum of its elements equal to 1
     */
    virtual void normalizeWeights();

    /**
     * @brief Publish the resulting data
     */
    virtual void publishEstimate() = 0;

  protected:
    // Ray casting implementation
    IRayCasting& mRayCasting;

    int mMaxParticles;
    std::vector<adx::data::Particlefld> mParticles;
    adx::data::Pose2f mPose;

    // Map descriptor
    Map_t mMapDescriptor;
    float* mDistanceMap = nullptr;

    // Cloud descriptor
    Cloud_t mCloudDescriptor;

    // init
    std::default_random_engine generator;

    // Motion model noise
    MotionModelNoise_t mOdomNoise;

    // Sensor Model Noise profile
    WeighingProfile_t mWeighingProfile;
};

#endif // PARTICLE_FILTER_PARTICLE_FILTER_HPP