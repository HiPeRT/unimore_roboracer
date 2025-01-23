#ifndef ADX_DATA_VECTOR3_HPP
#define ADX_DATA_VECTOR3_HPP

#include <Eigen/Core>

namespace adx {
namespace data {

template<typename Type, unsigned long Size>
struct Vector : public Eigen::Matrix<Type, Size, 1>
{
    /**
     * @brief Inherit constructor
     */
    using Eigen::Matrix<Type, Size, 1>::Matrix;
};

using Vector2f = Vector<float, 2>;
using Vector2d = Vector<double, 2>;
using Vector3f = Vector<float, 3>;
using Vector3d = Vector<double, 3>;

} // namespace data
} // namespace adx

#endif // ADX_DATA_VECTOR3_HPP
