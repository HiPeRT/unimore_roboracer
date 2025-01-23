#ifndef ADX_CURVE_SEGMENT_HPP
#define ADX_CURVE_SEGMENT_HPP

#include "adx_curve/curve.hpp"

namespace adx {
namespace curve {

class Segment : public Curve
{
  public:
    Segment(std::vector<float>* aXs, std::vector<float>* aYs, std::vector<float>* aSpeeds);
    Segment(std::vector<float>* aXs, std::vector<float>* aYs);
    ~Segment();

    void getCurveDistance(const Eigen::Vector3f& aPosition, float& aCurveDistance);
    void getPosition(const float aCurveDistance, Eigen::Vector3f& aPosition);

    void getSpeed(const float aCurveDistance, float& aSpeed);

  private:
    float mMaxDistance;
    std::vector<float>* mXs = nullptr;
    std::vector<float>* mYs = nullptr;
    std::vector<float>* mSpeeds = nullptr;
    std::vector<float> mSegmentDistance;
};

} // namespace curve
} // namespace adx


#endif // ADX_CURVE_SEGMENT_HPP
