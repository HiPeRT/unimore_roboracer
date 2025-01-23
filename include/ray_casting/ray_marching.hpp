#ifndef RAY_CASTING_RAY_MARCHING_HPP
#define RAY_CASTING_RAY_MARCHING_HPP

#include "ray_casting/ray_casting.hpp"

#include <omp.h>

/**
 * @brief RayMarching class
 *
 * This represents the CPU implementation of the RayMarching method.
 */
class RayMarching : public IRayCasting
{
    // inherit IRayCasting constructor
    using IRayCasting::IRayCasting;

  public:
    /**
     * @brief Constructs a RayMarching object
     */
    RayMarching();

    /**
     * @brief Computes a simulated point cloud for each particle using the ray marching algorithm.
     *
     * @param aParticles array containing particle states
     * @param aDistMap distance map
     * @param aMapData map information
     * @param aRaysAngles the angle for each ray to be casted
     * @param aRays the output array containing the casted rays for each particle
     */
    virtual void castRays(std::vector<adx::data::Particlefld>& aParticles,
                          float* aDistMap,
                          Map_t& aMapData,
                          std::vector<float>& aRaysAngles,
                          float* aRays) override;

  protected:
    // float x[MAX_PARTICLES];
    // float y[MAX_PARTICLES];
    // float yaw[MAX_PARTICLES];
    // float rays[MAX_PARTICLES * MAX_RAYS];
};

#endif // RAY_MARCHING_RAY_MARCHING_HPP
