#ifndef __SENSOR_MODEL_HPP__
#define __SENSOR_MODEL_HPP__

#include <adx_data/particle.hpp>
#include <vector>

#include "data_structures.hpp"

/**
 * @brief Generic Ray Casting interface
 *
 * This represents a common interface for all Ray Casting sensor model implementations
 */
class IRayCasting
{
  protected:
    std::vector<float> mSensorModelTable;
    float mSensorModelTableWidth;
    Cloud_t mCloudData;

    /**
     * @brief Computes a simulated point cloud for each particle.
     *
     * @param aParticles array containing particle states
     * @param aDistMap distance map
     * @param aMapData map information
     * @param aRaysAngles array of size MAX_RAYS containing the relative angle for each ray
     */
    virtual void castRays(std::vector<adx::data::Particlefld>& aParticles,
                            float* aDistMap,
                            Map_t& aMapData,
                            std::vector<float>& aRaysAngles,
                            float* aRays) = 0;

    /**
     * @brief Compute weights for each particle
     *
     * Computes the weight of each particle based on the
     * precomputed sensor model.
     *
     * @param aParticles the particles to be weighted
     * @param aRays the array of rays for each particle
     * @param aCloud current point cloud of the vehicle
     * @param aMapDescriptor the map descriptor
     */
    void computeWeights(std::vector<adx::data::Particlefld>& aParticles,
                        float* aRays,
                        std::vector<float>& aCloud,
                        Map_t& aMapDescriptor);

  public:
    IRayCasting();

    /**
     * @brief Precomputes the sensor model table
     *
     * @param aNoiseProfile sensor model table noise profile
     * @param aMaxMapRange Maximum possible map scan range (pixels)
     */
    void precomputeSensorModelTable(WeighingProfile_t aWeighingProfile, int aMaxMapRange);

    /**
     * @brief Casts the rays of each particle and computes its weight
     *
     * @param aParticles the particles to be weighted
     * @param aCloud the sensored point cloud
     * @param aDistMap the distance map of the environment
     * @param aMapDescriptor the map's descriptor
     * @param aRaysAngles the angle of each ray to be casted
     */
    void weighParticles(std::vector<adx::data::Particlefld>& aParticles,
                        std::vector<float>& aCloud,
                        float* aDistMap,
                        Map_t& aMapDescriptor,
                        std::vector<float>& aRaysAngles);

    /**
     * @brief Set the Cloud Descriptor
     *
     * @param aCloudData a cloud descriptor
     */
    void setCloudDescriptor(Cloud_t& aCloudData);
};

#endif //__SENSOR_MODEL_HPP__