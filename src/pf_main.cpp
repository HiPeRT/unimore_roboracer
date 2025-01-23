#include "particle_filter/particle_filter.hpp"
#include "particle_filter/particle_filter_node.hpp"
#include "ray_casting/ray_casting.hpp"
#include "ray_casting/ray_marching.hpp"

#ifdef FPGA_SUPPORT_ENABLED
#include "ray_casting/ray_marching_fpga.hpp"
#endif

#ifdef GPU_SUPPORT_ENABLED
//#include "ray_casting/ray_marching_gpu.hpp"
#endif

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // get parameter from ROS
    Implementation ray_casting_impl = Implementation::CPU;

    // consider using unique_ptr
    // std::unique_ptr<IRayCasting> ray_casting
    IRayCasting* ray_casting = nullptr;

    switch (static_cast<unsigned int>(ray_casting_impl)) {
        case static_cast<unsigned int>(Implementation::CPU): {
            // ray_casting = std::make_unique<RayMarching>()
            ray_casting = new RayMarching();
            break;
        }
        case static_cast<unsigned int>(Implementation::GPU): {
            // ray_casting = std::make_unique<RayMarchingGPU>()
            // ray_casting = new RayMarchingGPU();
            break;
        }
        case static_cast<unsigned int>(Implementation::FPGA): {
#ifdef FPGA_SUPPORT_ENABLED
            // ray_casting = std::make_unique<RayMarchingFPGA>()
            ray_casting = new RayMarchingFPGA();
#else
            std::cerr << "FPGA Implementation not available." <<
                         "Please ensure you have uio_map availeble to CMake." << std::endl;
#endif
            break;
        }
        default: {
            std::cerr << "Implementation unavailable" << std::endl;
            break;
        }
    }

    // consider using pointer and smart pointer in this class as well
    if (ray_casting != nullptr) {
        ParticleFilterNode<ParticleFilter> pfn(*ray_casting);
        pfn.run();
    }

    rclcpp::shutdown();
    return 0;
}
