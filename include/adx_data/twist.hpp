#ifndef ADX_DATA_TWIST_HPP
#define ADX_DATA_TWIST_HPP

#include <Eigen/Geometry>

#include "adx_data/covariance.hpp"
#include "adx_data/vector.hpp"

namespace adx {
namespace data {
namespace internal{

/**
 * @brief Velocity
 *
 * This represents velocity in free space broken into its linear and angular components.
 */
template<typename Type>
struct TwistBase
{
    /**
     * The 3D vector representing the linear velocity component
     */
    adx::data::Vector<Type, 3> linear = {};

    /**
     * The 3D vector representing the angular velocity component
     */
    adx::data::Vector<Type, 3> angular = {};
    /**
     * @brief Construct a new Twist object
     *
     * Default constructor, this should just zero-initialize everything
     */
    TwistBase() = default;
};

// TwistHeader
template<typename Type, bool HasHeader>
struct TwistHeader : virtual public TwistBase<Type>
{
    /**
     * @brief inherit constructors
     */
    using TwistBase<Type>::TwistBase;

    /**
     * @brief Construct a new TwistHeader object
     */
    TwistHeader() = default;
};

template<typename Type>
struct TwistHeader<Type, true> 
    : virtual public Header
    , virtual public TwistBase<Type>
{
    /**
     * @brief inherit constructors
     */
    using TwistBase<Type>::TwistBase;

    /**
     * @brief Construct a new TwistHeader object
     */
    TwistHeader() = default;
};

// TwistCovariance
template<typename Type, bool HasCovariance>
struct TwistCovariance: virtual public TwistBase<Type>
{
    /**
     * @brief inherit constructors
     */
    using TwistBase<Type>::TwistBase;

    /**
     * @brief Construct a new TwistCovariance object
     */
    TwistCovariance() = default;
};

template<typename Type>
struct TwistCovariance<Type, true> : virtual public TwistBase<Type>
{
    /**
     * @brief Inherit constructors
     */
    using TwistBase<Type>::TwistBase;

    /**
     * @brief Covariace Matrix
     */
    Covarianced<6> twist_covariance = {};

    /**
     * @brief Construct a new TwistCovariance object
     */
    TwistCovariance() = default;
};

// TwistHeaderCovariance
template<typename Type, bool HasHeader, bool HasCovariance>
struct TwistHeaderCovariance
    : public TwistHeader<Type, HasHeader>
    , virtual public TwistBase<Type>
    , public TwistCovariance<Type, HasCovariance>
{
    /**
     * @brief inherit constructors
     */
    using TwistBase<Type>::TwistBase;
    using TwistHeader<Type, HasHeader>::TwistHeader;
    using TwistCovariance<Type, HasCovariance>::TwistCovariance;

    /**
     * @brief Construct a new TwistHeaderCovariance object
     */
    TwistHeaderCovariance() = default;
};

template<typename Type>
struct TwistHeaderCovariance<Type, true, false>
    : public TwistHeader<Type, true>
    , virtual public TwistBase<Type>
    , public TwistCovariance<Type, false>
{
    /**
     * @brief inherit constructors
     */
    using TwistBase<Type>::TwistBase;
    using TwistHeader<Type, true>::TwistHeader;
    using TwistCovariance<Type, false>::TwistCovariance;

    /**
     * @brief Construct a new TwistHeaderCovariance object
     */
    TwistHeaderCovariance() = default;
};

template<typename Type>
struct TwistHeaderCovariance<Type, true, true>
    : public TwistHeader<Type, true>
    , virtual public TwistBase<Type>
    , public TwistCovariance<Type, true>
{
    /**
     * @brief inherit constructors
     */
    using TwistBase<Type>::TwistBase;
    using TwistHeader<Type, true>::TwistHeader;
    using TwistCovariance<Type, true>::TwistCovariance;

    /**
     * @brief Construct a new TwistHeaderCovariance object
     */
    TwistHeaderCovariance() = default;
};
// TODO: is no header with covariance needed?

} // namespace internal


template<typename Type, bool HasHeader, bool HasCovariance>
struct Twist : public internal::TwistHeaderCovariance<Type, HasHeader, HasCovariance>
{
    /**
     * @brief inherit constructors
     */
    using internal::TwistHeaderCovariance<Type, HasHeader, HasCovariance>::TwistHeaderCovariance;
    /**
     * @brief Construct a new TwistHeaderCovariance object
     */
    Twist() = default;
};

template<typename Type, bool HasCovariance>
struct Twist<Type, false, HasCovariance>
    : public internal::TwistHeaderCovariance<Type, false, HasCovariance>
{
     /**
     * @brief inherit constructors
     */
    using internal::TwistHeaderCovariance<Type, false, HasCovariance>::TwistHeaderCovariance;
    /**
     * @brief Construct a new TwistHeaderCovariance object
     */
    Twist() = default;
};

using Twistd = Twist<double, true, true>;
using Twistf = Twist<float, true, true>;


} // namespace data
} // namespace adx

#endif // ADX_DATA_TWIST_HPP
