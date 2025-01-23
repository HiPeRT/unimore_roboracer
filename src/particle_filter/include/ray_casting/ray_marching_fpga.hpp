#ifndef RAY_CASTING_RAY_MARCHING_FPGA_HPP
#define RAY_CASTING_RAY_MARCHING_FPGA_HPP

#include <uio_map.hpp>

#include "ray_casting/ray_marching.hpp"

/**
 * @brief UIO devices enumerator
 */
enum uio_devices : unsigned int
{
    RM = 4,
    X_BUF = 5,
    Y_BUF = 6,
    YAW_BUF = 7,
    RAYS_BUF = 8,
    MAP_BUF = 9,
    ANGLES_BUF = 10
};

/**
 * @brief RayMarching with FPGA-accelerated methods
 */
class RayMarchingFPGA : public RayMarching
{
  public:
    static const int MAX_PARTICLES = 2000;
    static const int MAX_RAYS = 60;

    using RayMarching::RayMarching;

    RayMarchingFPGA();

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
    // ray marching IP
    UioMap mRM;

    // x memory buffer
    UioMap mXBuffer;

    // y memory buffer
    UioMap mYBuffer;

    // yaw memory buffer
    UioMap mYawBuffer;

    // rays memory buffer
    UioMap mRaysBuffer;

    // map memory buffer
    UioMap mMapBuffer;

    // angles memory buffer
    UioMap mAnglesBuffer;

    // true if the map has been loaded into the map buffer
    bool mMapLoaded = false;
};

#endif // RAY_CASTING_RAY_MARCHING_FPGA_HPP
