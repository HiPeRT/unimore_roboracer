#ifndef ADX_DATA_QUATERNION_HPP
#define ADX_DATA_QUATERNION_HPP

#include <Eigen/Geometry>

namespace adx {
namespace data {

/**
 * @brief Orientation
 *
 * This represents an orientation in free space in quaternion form.
 */
template<typename Type>
struct Quaternion : public Eigen::Quaternion<Type>
{

    /**
     * @brief Inherit constructors
     */
    using Eigen::Quaternion<Type>::Quaternion;

    /**
     * @brief set orientation to Roll, Pitch, Yaw
     *
     * @param aRoll the rotation around the X axis
     * @param aPitch the rotation around the Y axis
     * @param aYaw the rotation around the Z axis
     */
    void setRPY(const Type& aRoll, const Type& aPitch, const Type& aYaw)
    {
        Eigen::Quaternion<Type>::operator=(
          Eigen::AngleAxis<Type>(aRoll, Eigen::Matrix<Type, 3, 1>::UnitX()) *
          Eigen::AngleAxis<Type>(aPitch, Eigen::Matrix<Type, 3, 1>::UnitY()) *
          Eigen::AngleAxis<Type>(aYaw, Eigen::Matrix<Type, 3, 1>::UnitZ()));
    }

    /**
     * @brief return the Roll, Pitch and Yaw
     *
     * @return Eigen::Matrix<Type, 3, 1> Vector containing roll, pitch and yaw
     */
    Eigen::Matrix<Type, 3, 1> getRPY() const
    {
        return Eigen::Quaternion<Type>::toRotationMatrix().eulerAngles(0, 1, 2);
    }
};

using Quaternionf = Quaternion<float>;
using Quaterniond = Quaternion<double>;

} // namespace data
} // namespace adx

#endif // ADX_DATA_QUATERNION_HPP
