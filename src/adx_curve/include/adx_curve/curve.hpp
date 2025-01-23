#ifndef CURVE_HPP
#define CURVE_HPP

#include "Eigen/Core"

#include "adx_curve/vector.hpp"

class Curve
{
  public:
    virtual ~Curve() {}

    virtual void getCurveDistance(const Eigen::Vector3f& aPosition, float& aCurveDistance) = 0;
    virtual void getPosition(const float aCurveDistance, Eigen::Vector3f& aPosition) = 0;

    virtual void getSpeed(const float aCurveDistance, float& aSpeed) = 0;
};

#endif // CURVE_HPP
