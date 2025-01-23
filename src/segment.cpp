#include <cmath>

#ifdef __linumXs_
using namespace std;
#endif

#include "adx_curve/segment.hpp"

using namespace adx::curve;

Segment::Segment(std::vector<float>* aXs, std::vector<float>* aYs, std::vector<float>* aSpeeds)
  : mXs(aXs)
  , mYs(aYs)
  , mSpeeds(aSpeeds)
{
    mSegmentDistance.push_back(0);
    for (unsigned int i = 1; i < aXs->size(); ++i) {
        mSegmentDistance.push_back(hypot((*aXs)[i] - (*aXs)[i - 1], (*aYs)[i] - (*aYs)[i - 1]) +
                                    mSegmentDistance[i - 1]);
    }
}

Segment::Segment(std::vector<float>* aXs, std::vector<float>* aYs)
  : mXs(aXs)
  , mYs(aYs)
{
    mSegmentDistance.push_back(0);
    for (unsigned int i = 1; i < aXs->size(); ++i) {
        mSegmentDistance.push_back(hypot((*aXs)[i] - (*aXs)[i - 1], (*aYs)[i] - (*aYs)[i - 1]) +
                                    mSegmentDistance[i - 1]);
    }
}

Segment::~Segment() {}

void Segment::getPosition(float aDistance, Eigen::Vector3f& aPosition)
{
    // binary search
    static unsigned int start, end, i;
    start = 0;
    end = mSegmentDistance.size() - 1;

    while (start != end - 1) {
        i = (start + end) / 2;

        if (mSegmentDistance[i] < aDistance)
            start = i;
        else
            end = i;
    }

    // lerp
    float fraction = std::abs((aDistance - mSegmentDistance[start]) /
                               (mSegmentDistance[end] - mSegmentDistance[start]));

    aPosition = { (*mXs)[start] + fraction * ((*mXs)[end] - (*mXs)[start]),
                 (*mYs)[start] + fraction * ((*mYs)[end] - (*mYs)[start]),
                 0 };
}

void Segment::getSpeed(const float aCurveDistance, float& aSpeed)
{
    aSpeed = (*mSpeeds)[((unsigned int)std::ceil(((aCurveDistance) / mSegmentDistance.back()) *
                                                mSpeeds->size())) %
                       mSpeeds->size()];
}

void Segment::getCurveDistance(const Eigen::Vector3f& aPosition, float& aCurveDistance)
{
    Eigen::Vector3f p0, p1;
    float d_best, d1, s1;
    float s_best, d_temp;
    float interval = 0.05;

    s_best = aCurveDistance;
    d_temp = hypot(aPosition.x() - p0.x(), aPosition.y() - p0.y());

    if (s_best >= mSegmentDistance[mSegmentDistance.size() - 1] - interval)
        s_best = 0;

    getPosition(s_best, p0);
    d_best = d_temp;

    // if current distance is >2m relocate globally
    if (d_best > 2.0) {
        int linspace_size = 500;
        float step = mSegmentDistance.back() / linspace_size;
        for (float i = 0; i < mSegmentDistance.back(); i += step) {
            getPosition(i, p1);
            d1 = hypot(aPosition.x() - p1.x(), aPosition.y() - p1.y());
            if (d1 < d_best) {
                d_best = d1;
                s_best = i;
            }
        }
    } else { // locate locally
        s1 = s_best;
        d1 = d_temp;

        while (s_best < mSegmentDistance.back()) {
            s1 = s1 + interval;
            getPosition(s1, p1);
            d1 = hypot(aPosition.x() - p1.x(), aPosition.y() - p1.y());

            if (d1 <= d_best) {
                s_best = s1;
                d_best = d1;
            } else
                break;

            d_temp = d1;
        }
    }

    aCurveDistance = s_best;
}
