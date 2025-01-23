#ifndef ADX_DATA_PATH_HPP
#define ADX_DATA_PATH_HPP

#include <vector>

#include "adx_data/header.hpp"
#include "adx_data/pose.hpp"
#include "adx_data/vector.hpp"

namespace adx {
namespace data {

/**
 * @brief Path for a robot to follow
 *
 * A series of Pose representing a Path for robot navigation
 */
struct Path : virtual public Header
{
    using PoseType = Pose<double, 3, true, false>;

    /**
     * The series of poses that are used to represent the path
     */
    // std::vector<Pose3d> poses;
    std::vector<PoseType> poses;

    /**
     * @brief Construct a new Path object
     *
     * Default constructor, this should just zero-initialize the poses
     */
    Path() = default;
};

/**
 * @brief Plan for a robot to follow
 *
 * This is essentially just an Path that uses Point instead of Pose
 * and stores a reference speed for each point.
 * This is useful in racing environments.
 */
struct Plan : virtual public Header
{
    /**
     * The series of points that are used to represent the path
     */
    std::vector<Point3f> positions;

    /**
     * The reference speeds associated with each of the points
     */
    std::vector<adx::data::Vector3f> speeds;

    /**
     * @brief Construct a new Plan object
     *
     * Default constructor, this should just initialize the empty vectors
     */
    Plan() = default;
};

} // namespace data
} // namespace adx

#endif // ADX_DATA_PATH_HPP
