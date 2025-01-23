#pragma once
//#include "ray_marching.hpp"

#include "data_structures.hpp"
#include <cuda.h>
#include <cuda_runtime.h>

class cRayMarching
{
  public:
    void init(Particle_t* particles,
              float* distMap,
              Cloud_t* cloud,
              Map_t* map,
              int n_particles,
              float* rays_angle);
    /**
     * Method to generate virtual rays (2D lidar scan) at the position of each particle passed as
     * parameter.
     * @param particles     input particles.
     * @param rays          output generated rays.
     */
    void calculateRays(Particle_t* particles, int n_particles, float *rays);
    /**
     * Method to deallocate memory.
     */
    void close();

  private:
    cudaStream_t stream;
    float* d_distGrid;
    float* d_rays;
    Particle_t* d_particles;
    Map_t* d_map;
    Cloud_t* d_cloud;
    float* d_raysAngle;
};