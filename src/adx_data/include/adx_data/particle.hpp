#ifndef ADX_DATA_PARTICLE_HPP
#define ADX_DATA_PARTICLE_HPP

#include "adx_data/pose.hpp"

namespace adx {
namespace data {

namespace internal {

/**
 * @brief A generic 1-dimensional weight
 *
 * @tparam Type The type of the weight
 */
template<typename Type>
struct Weight
{
    /**
     * The Type value representing the weight
     */
    Type weight;

    /**
     * @brief Construct a new Weight object
     *
     * Default constructor, this should just zero-initialize a Weight
     */
    Weight() = default;
};

} // namespace internal

/**
 * @brief A weighted estimate of robot pose
 *
 * Pose represents the estimated position and orientation of the robot.
 * Weight represents the estimated weight of this particle
 */
template<typename PoseType, typename WeightType>
struct Particle
  : public Pose<PoseType, 2, true, false>
  , public internal::Weight<WeightType>
{
    /**
     * @brief inherit Pose constructors
     */
    using Pose<PoseType, 2, true, false>::Pose;

    /**
     * @brief inherit Weight constructors
     */
    using internal::Weight<WeightType>::Weight;

    /**
     * @brief Construct a new Particle object
     *
     * Default constructor, this should just zero-initialize a Particle
     */
    Particle() = default;
};

using Particlef  = Particle<float, float>;
using Particled  = Particle<double, double>;
using Particleld = Particle<long double, long double>;
using Particlefd  = Particle<float, double>;
using Particlefld  = Particle<float, long double>;
using Particledld  = Particle<double, long double>;

} // namespace data
} // namespace adx

#endif // ADX_DATA_PARTICLE_HPP