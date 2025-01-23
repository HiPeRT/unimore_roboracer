#ifndef ADX_DATA_OBSTACLE_HPP
#define ADX_DATA_OBSTACLE_HPP

#include "adx_data/covariance.hpp"
#include "adx_data/pose.hpp"
#include "adx_data/twist.hpp"
#include "adx_data/vector.hpp"

namespace adx {
namespace data {

/**
 * @brief Cube obstacle
 *
 * This struct represents a cube obstacle.
 */
struct Obstacle
  : public Pose3d
  , public Twistd
{
    /**
     * The cube half-lenghts, the linear combination of position and size gives us the cube
     * vertices
     */
    adx::data::Vector3d size = {};

    /**
     * @brief Construct a new Obstacle object
     *
     * The default constructor should just zero-initialize everything.
     */
    Obstacle() = default;
};

} // nmespace data
} // namespace adx

#endif // ADX_DATA_OBSTACLE_HPP
