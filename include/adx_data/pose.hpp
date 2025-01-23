#ifndef ADX_DATA_POSE_HPP
#define ADX_DATA_POSE_HPP

#include "adx_data/header.hpp"
#include "adx_data/covariance.hpp"
#include "adx_data/point.hpp"
#include "adx_data/quaternion.hpp"

namespace adx {
namespace data {
namespace internal {

constexpr unsigned long covariance_dimension[4] = { 0, 1, 3, 6 };

/**
 * @brief PoseOrientation
 *
 * A representation of pose orientation in free space.
 */

template<typename Type, unsigned long Size>
struct PoseOrientation
{
    static_assert(Size >= 2, "Minimum size for PoseOrientation is 2");
    static_assert(Size <= 3, "Maximum size for PoseOrientation is 3");

    /**
     * @brief The orientation part of the Pose
     */
    adx::data::Quaternion<Type> orientation = { static_cast<Type>(1),
                                                static_cast<Type>(0),
                                                static_cast<Type>(0),
                                                static_cast<Type>(0) };

    /**
     * @brief Construct a new PoseOrientation object
     *
     * Default constructor, this should just zero-initialize everything
     */
    PoseOrientation() = default;

};

template<typename Type>
struct PoseOrientation<Type, 2>
{
    /**
     * @brief Construct a new PoseOrientation object
     */
    PoseOrientation() = default;

    /**
     * @brief yaw accessor
     * @return Type& mutable reference to yaw variable
     */
    Type& yaw() { return mYaw; }

    /**
     * @brief yaw accessor
     * @return const Type& immutable reference to yaw variable
     */
    const Type& yaw() const { return mYaw; }

    Quaternion<Type> toQuaternion() const
    {
        Quaternion<Type> adx_quaternion;

        adx_quaternion.setRPY(0.0, 0.0, mYaw);

        return adx_quaternion;
    }

  private:
    Type mYaw;
};

template<typename Type, unsigned long Size>
struct PoseBase
  : public Point<Type, Size>
  , public PoseOrientation<Type, Size>
{
    static_assert(Size >= 2, "Minimum size for PoseBase is 2");
    static_assert(Size <= 3, "Maximum size for PoseBase is 3");

    /**
     * @brief inherit Point constructors
     */
    using Point<Type, Size>::Point;
    using PoseOrientation<Type, Size>::PoseOrientation;

    /**
     * @brief Construct a new PoseBase object
     */
    PoseBase() = default;
};

template<typename Type, unsigned long Size, bool HasHeader>
struct PoseHeader : virtual public PoseBase<Type, Size>
{
    /**
     * @brief inherit constructors
     */
    using PoseBase<Type, Size>::PoseBase;

    /**
     * @brief Construct a new PoseHeader object
     */
    PoseHeader() = default;

};

template<typename Type, unsigned long Size>
struct PoseHeader<Type, Size, true>
  : virtual public Header
  , virtual public PoseBase<Type, Size>
{
    /**
     * @brief inherit constructors
     */
    using PoseBase<Type, Size>::PoseBase;

    /**
     * @brief Construct a new PoseHeader object
     */
    PoseHeader() = default;
};

template<typename Type, unsigned long Size, bool HasCovariance>
struct PoseCovariance : virtual public PoseBase<Type, Size>
{

    /**
     * @brief inherit constructors
     */
    using PoseBase<Type, Size>::PoseBase;

    /**
     * @brief Construct a new PoseCovariance object
     */
    PoseCovariance() = default;
};

template<typename Type, unsigned long Size>
struct PoseCovariance<Type, Size, true> : virtual public PoseBase<Type, Size>
{
    /**
     * @brief inherit constructors
     */
    using PoseBase<Type, Size>::PoseBase;

    /**
     * @brief The Covariance matrix
     * For 2D pose: x, y, z-rotation;
     * For 3D pose: x, y, z, x-rotation, y-rotation, z-rotation.
     */
    Covariance<Type, internal::covariance_dimension[Size]> pose_covariance = {};

    /**
     * @brief Construct a new PoseCovariance object
     */
    PoseCovariance() = default;
};

template<typename Type, unsigned long Size, bool HasHeader, bool HasCovariance>
struct PoseHeaderCovariance
  : public PoseHeader<Type, Size, HasHeader>
  , virtual public PoseBase<Type, Size>
  , public PoseCovariance<Type, Size, HasCovariance>
{
    /**
     * @brief inherit constructors
     */
    using PoseHeader<Type, Size, HasHeader>::PoseHeader;
    using PoseBase<Type, Size>::PoseBase;
    using PoseCovariance<Type, Size, HasCovariance>::PoseCovariance;

    /**
     * @brief Construct a new PoseHeaderCovariance object
     */
    PoseHeaderCovariance() = default;
};

template<typename Type, unsigned long Size>
struct PoseHeaderCovariance<Type, Size, true, false>
  : public PoseHeader<Type, Size, true>
  , virtual public PoseBase<Type, Size>
  , public PoseCovariance<Type, Size, false>
{
    /**
     * @brief inherit constructors
     */
    using PoseBase<Type, Size>::PoseBase;
    using PoseHeader<Type, Size, true>::PoseHeader;
    using PoseCovariance<Type, Size, false>::PoseCovariance;

    /**
     * @brief Construct a new PoseHeaderCovariance object
     */
    PoseHeaderCovariance() = default;
};

template<typename Type, unsigned long Size>
struct PoseHeaderCovariance<Type, Size, true, true>
  : public PoseHeader<Type, Size, true>
  , virtual public PoseBase<Type, Size>
  , public PoseCovariance<Type, Size, true>
{
    /**
     * @brief inherit constructors
     */
    using PoseHeader<Type, Size, true>::PoseHeader;
    using PoseCovariance<Type, Size, true>::PoseCovariance;

    /**
     * @brief Construct a new PoseHeaderCovariance object
     */
    PoseHeaderCovariance() = default;
};

} // namespace internal

template<typename Type, unsigned long Size, bool HasHeader, bool HasCovariance>
struct Pose : public internal::PoseHeaderCovariance<Type, Size, HasHeader, HasCovariance>
{
    static_assert(Size >= 2, "Minimum size for Pose is 2");
    static_assert(Size <= 3, "Maximum size for Pose is 3");

    /**
     * @brief inherit Point constructors
     */
    using internal::PoseHeaderCovariance<Type, Size, HasHeader, HasCovariance>::
      PoseHeaderCovariance;

    /**
     * @brief Construct a new Pose object
     */
    Pose() = default;
};

template<typename Type, unsigned long Size, bool HasCovariance>
struct Pose<Type, Size, false, HasCovariance>
  : public internal::PoseHeaderCovariance<Type, Size, false, HasCovariance>
{
    static_assert(Size >= 2, "Minimum size for Pose is 2");
    static_assert(Size <= 3, "Maximum size for Pose is 3");

    /**
     * @brief inherit Point constructors
     */
    using internal::PoseHeaderCovariance<Type, Size, false, HasCovariance>::PoseHeaderCovariance;

    /**
     * @brief Construct a new Pose object
     */
    Pose() = default;
};

using Pose2f = Pose<float, 2, true, true>;
using Pose2d = Pose<double, 2, true, true>;
using Pose3f = Pose<float, 3, true, true>;
using Pose3d = Pose<double, 3, true, true>;

} // namespace data
} // namespace adx

#endif // ADX_DATA_POSE_HPP
