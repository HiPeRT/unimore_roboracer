#include <cuda.h>
#include <cuda_runtime.h>

#include "ray_marching/ray_marching_gpu.hpp"

__global__ void cuda_ray_marching(Particle_t* particles,
                                  float* distMap,
                                  float* rays_angle,
                                  int n_particles,
                                  Cloud_t* cloud,
                                  Map_t* map,
                                  float* rays)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int nCast = n_particles * N_RAYS_DS;
    if (idx >= nCast)
        return;
    int pIDX = (float)idx / float(N_RAYS_DS);
    int aIDX = fmodf(idx, N_RAYS_DS); // idx % angleToCast;

    // int iix = int(aIDX * angleDownsample);
    float angle =
      (particles[pIDX].yaw + cloud->angleMin +
       rays_angle[aIDX]); // angleMin) + (float(aIDX) * float(angleDownsample)) * angleIncrement;

    // generate ray with rayMarching
    float rayPoseX = particles[pIDX].x;
    float rayPoseY = particles[pIDX].y;

    float t = 0.0f;
    float out = cloud->maxRange;
    while (t < cloud->maxRayIteration) {
        int c = (int)((map->opp_originX - rayPoseX) / map->map_resolution);
        int r = (int)((map->opp_originY + rayPoseY) / map->map_resolution);

        if (c < 0 || c >= map->map_width || r < 0 || r > map->map_height) {
            out = cloud->maxRange;
            break;
        }
        int i = r * map->map_width + c;
        float distance = distMap[i];
        rayPoseX += distance * std::cos(angle);
        rayPoseY += distance * std::sin(angle);

        if (distance <= map->map_resolution) {
            float xd = rayPoseX - particles[pIDX].x;
            float yd = rayPoseY - particles[pIDX].y;
            out = sqrtf(xd * xd + yd * yd);
            break;
        }

        t += fmaxf(distance * 0.999f, 1.0);
    }
    rays[idx] = out;
}

// turn this into a constructor, move cudaMemcpyAsync?
void RayMarchingGPU::init(Particle_t* particles,
                        float* distMap,
                        Cloud_t* cloud,
                        Map_t* map,
                        int n_particles,
                        float* rays_angle)
{
    cudaStreamCreate(&stream);

    cudaMalloc((void**)&d_distGrid, sizeof(float) * map->map_height * map->map_width);
    cudaMalloc((void**)&d_particles, sizeof(Particle_t) * n_particles);
    cudaMalloc((void**)&d_cloud, sizeof(Cloud_t));
    cudaMalloc((void**)&d_map, sizeof(Map_t));
    cudaMalloc((void**)&d_raysAngle, sizeof(float) * N_RAYS_DS);
    cudaMalloc((void**)&d_rays, sizeof(float) * n_particles * N_RAYS_DS);

    cudaMemcpyAsync(d_cloud, cloud, sizeof(Cloud_t), cudaMemcpyHostToDevice, stream);
    cudaMemcpyAsync(d_map, map, sizeof(Map_t), cudaMemcpyHostToDevice, stream);
    cudaMemcpyAsync(d_distGrid,
                    distMap,
                    sizeof(float) * map->map_height * map->map_width,
                    cudaMemcpyHostToDevice,
                    stream);
    cudaMemcpyAsync(
      d_raysAngle, &rays_angle[0], sizeof(float) * N_RAYS_DS, cudaMemcpyHostToDevice, stream);
}

void RayMarchingGPU::castRays(Particle_t* particles, int n_particles, float* rays)
{
    // copy to gpu
    cudaMemcpyAsync(
      d_particles, &particles[0], sizeof(Particle_t) * n_particles, cudaMemcpyHostToDevice, stream);

    int nCast = n_particles * N_RAYS_DS;
    int blockSize = 192;
    int numBlocks = (nCast + blockSize - 1) / blockSize;

    cuda_ray_marching<<<numBlocks, blockSize, 0, stream>>>(
      d_particles, d_distGrid, d_raysAngle, n_particles, d_cloud, d_map, d_rays);

    cudaMemcpyAsync(
      &rays[0], d_rays, sizeof(float) * n_particles * N_RAYS_DS, cudaMemcpyDeviceToHost, stream);
    cudaStreamSynchronize(stream);
}

void RayMarchingGPU::close()
{
    cudaFree(d_distGrid);
    cudaFree(d_particles);
    cudaFree(d_cloud);
    cudaFree(d_map);
}