#ifndef RAY_CASTING_RAY_MARCHING_GPU_HPP
#define RAY_CASTING_RAY_MARCHING_GPU_HPP

#include "ray_casting/ray_marching.hpp"

/**
 * @brief RayMarching class with GPU-accelerated methods
 */
class RayMarchingGPU : public RayMarching
{
  public:
    RayMarchingGPU();

    /**
     * @brief Computes a simulated point cloud for each particle on a GPU accelerator.
     *
     * @param aParticles array containing particle states
     * @param aDistMap distance map
     * @param aMapData map information
     * @param aRaysAngles array of size MAX_RAYS containing the relative angle for each ray
     */
    virtual void castRays(Particle_t* particles,
                          float* distMap,
                          Map_t* map,
                          Cloud_t* cloud,
                          int n_particles,
                          float* rays_angle) override;
};

#endif // RAY_MARCHING_RAY_MARCHING_GPU_HPP
