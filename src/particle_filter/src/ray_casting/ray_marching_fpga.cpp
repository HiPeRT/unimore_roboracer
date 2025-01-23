#include "ray_casting/ray_marching_fpga.hpp"
#include "ray_casting/xrm.hpp"
#include <cstring>
#include <ctime>

RayMarchingFPGA::RayMarchingFPGA()
  : RayMarching()
  , mMapLoaded(false)
{
    while (uio_initialize(&mRM, uio_devices::RM)) {
        std::cout << "Init Error RM" << std::endl;
        uio_release(&mRM);
    }

    while (uio_initialize(&mXBuffer, uio_devices::X_BUF)) {
        std::cout << "Init Error X" << std::endl;
        uio_release(&mXBuffer);
    }

    while (uio_initialize(&mYBuffer, uio_devices::Y_BUF)) {
        std::cout << "Init Error Y" << std::endl;
        uio_release(&mYBuffer);
    }

    while (uio_initialize(&mYawBuffer, uio_devices::YAW_BUF)) {
        std::cout << "Init Error YAW" << std::endl;
        uio_release(&mYawBuffer);
    }

    while (uio_initialize(&mRaysBuffer, uio_devices::RAYS_BUF)) {
        std::cout << "Init Error RAYS" << std::endl;
        uio_release(&mRaysBuffer);
    }

    while (uio_initialize(&mMapBuffer, uio_devices::MAP_BUF)) {
        std::cout << "Init Error MAP" << std::endl;
        uio_release(&mMapBuffer);
    }

    while (uio_initialize(&mAnglesBuffer, uio_devices::ANGLES_BUF)) {
        std::cout << "Init Error ANGLES" << std::endl;
        uio_release(&mAnglesBuffer);
    }
}

void RayMarchingFPGA::castRays(std::vector<adx::data::Particlefld>& aParticles,
                           float* aDistMap,
                           Map_t& aMapData,
                           std::vector<float>& aRaysAngles,
                           float* aRays)
{
    double orig_x = aMapData.opp_originX;
    double orig_y = aMapData.opp_originY;
    double map_resolution = aMapData.map_resolution;

    // check for oversized rays or particles buffers

    // if (!mMapLoaded) {
    //     for (int i = 0; i < aMapData.map_height * aMapData.map_width; ++i) {
    //         distanceMap_d[i] = aDistMap[i];
    //     }
    // }

    if (!mMapLoaded) {
        for (int i = 0; i < aMapData.map_height * aMapData.map_width; ++i) {
            ((float*)mMapBuffer.addr)[i] = aDistMap[i];
        }
    }

    for (size_t i = 0; i < aParticles.size(); ++i) {
        ((float*)mXBuffer.addr)[i] = aParticles[i].position.x();
    }

    for (size_t i = 0; i < aParticles.size(); ++i) {
        ((float*)mYBuffer.addr)[i] = aParticles[i].position.y();
    }

    for (size_t i = 0; i < aRaysAngles.size(); ++i) {
        ((float*)mAnglesBuffer.addr)[i] = aRaysAngles[i];
    }

    for (size_t i = 0; i < aParticles.size(); ++i) {
        ((float*)mYawBuffer.addr)[i] = aParticles[i].yaw();
    }

    if (!mMapLoaded) {
        XRmk_Set_x(&mRM, (uint32_t)mXBuffer.phy);
        XRmk_Set_y(&mRM, (uint32_t)mYBuffer.phy);
        XRmk_Set_yaw(&mRM, (uint32_t)mYawBuffer.phy);
        XRmk_Set_rays(&mRM, (uint32_t)mRaysBuffer.phy);
        XRmk_Set_rays_angle(&mRM, (uint32_t)mAnglesBuffer.phy);
        XRmk_Set_distMap(&mRM, (uint32_t)mMapBuffer.phy);
        XRmk_Set_orig_x(&mRM, (*(uint32_t*)&orig_x));
        XRmk_Set_orig_y(&mRM, (*(uint32_t*)&orig_y));
        XRmk_Set_map_resolution(&mRM, (*(uint32_t*)&map_resolution));
        XRmk_Set_map_height(&mRM, (uint32_t)aMapData.map_height);
        XRmk_Set_map_width(&mRM, (uint32_t)aMapData.map_width);
        XRmk_Set_rm_mode(&mRM, 0);

        XRmk_Start(&mRM);
        while (!XRmk_IsDone(&mRM))
            ;

        mMapLoaded = true;
        XRmk_Set_rm_mode(&mRM, 1);
    }

    XRmk_Set_n_particles(&mRM, (uint32_t)aParticles.size());

    XRmk_Start(&mRM);

    // TODO: can this not be an active spin
    while (!XRmk_IsDone(&mRM))
        ;

    for (size_t i = 0; i < aParticles.size(); ++i) {
        for (size_t j = 0; j < aRaysAngles.size(); ++j) {
            aRays[i * aRaysAngles.size() + j] = ((float*)mRaysBuffer.addr)[i * aRaysAngles.size() + j];
        }
    }
}