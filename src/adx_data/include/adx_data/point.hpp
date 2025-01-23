#ifndef ADX_DATA_POINT_HPP
#define ADX_DATA_POINT_HPP

#include "adx_data/vector.hpp"

namespace adx {
namespace data {

/**
 * @brief 3D position
 *
 * This contains the position of a point in free space.
 */
template<typename Type, unsigned long Size>
struct Point
{
    static_assert(Size >= 2, "Minimum size for Point is 2");
    static_assert(Size <= 3, "Maximum size for Point is 3");
    /**
     * The 3D vector representing the 3D position
     */
    adx::data::Vector<Type, Size> position = {};

    /**
     * @brief Construct a new Point object
     *
     * Default constructor, this should just zero-initialize the position
     */
    Point() = default;
};

using Point2f = Point<float, 2>;
using Point2d = Point<double, 2>;
using Point3f = Point<float, 3>;
using Point3d = Point<double, 3>;

} // nmespace data
} // namespace adx

#endif // ADX_DATA_POINT_HPP
