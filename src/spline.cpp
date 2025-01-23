#include <cmath>
#include <iostream>
#include <stdexcept>

#include "adx_curve/spline.hpp"

using namespace adx::curve;

Spline::Spline(std::vector<float>& aXs, std::vector<float>& aYs)
  : mXs(aXs)
  , mAs(aYs)
{
    if (aXs.size() != aYs.size()) {
        std::cout << "[WARNING] Input vectors have different sizes" << std::endl;
    }
    std::vector<float> h;
    vectorDifference(aXs, h);

    Eigen::MatrixXf A;
    Eigen::VectorXf b, c_eigen;
    initCoefficientsMatrix(A, h);
    initConstantTermsVector(b, h);
    c_eigen = A.partialPivLu().solve(b);

    float* c_pointer = c_eigen.data();
    mCs.assign(c_pointer, c_pointer + c_eigen.rows());

    for (unsigned int i = 0; i < mXs.size() - 1; i++) {
        mDs.push_back((mCs[i + 1] - mCs[i]) / (3.0 * h[i]));
        mBs.push_back((mAs[i + 1] - mAs[i]) / h[i] - h[i] * (mCs[i + 1] + 2 * mCs[i]) / 3.0);
    }
}

float Spline::eval(float aT)
{
    aT = std::max(mXs.front(), std::min(mXs.back(), aT));

    int seg_id = bisect(aT, 0, mXs.size());
    float dx = aT - mXs[seg_id];
    return mAs[seg_id] + mBs[seg_id] * dx + mCs[seg_id] * dx * dx + mDs[seg_id] * dx * dx * dx;
}

float Spline::evalDerivative(float aT)
{
    aT = std::max(mXs.front(), std::min(mXs.back(), aT));

    int seg_id = bisect(aT, 0, mXs.size() - 1);
    float dx = aT - mXs[seg_id];
    return mBs[seg_id] + 2 * mCs[seg_id] * dx + 3 * mDs[seg_id] * dx * dx;
}

float Spline::evalDerivative2(float aT)
{
    aT = std::max(mXs.front(), std::min(mXs.back(), aT));

    int seg_id = bisect(aT, 0, mXs.size());
    float dx = aT - mXs[seg_id];
    return 2 * mCs[seg_id] + 6 * mDs[seg_id] * dx;
}

void Spline::initCoefficientsMatrix(Eigen::MatrixXf& A, std::vector<float>& h)
{
    A = Eigen::MatrixXf::Zero(mXs.size(), mXs.size());
    A(0, 0) = 1;
    for (unsigned int i = 0; i < mXs.size() - 1; i++) {
        if (i != mXs.size() - 2)
            A(i + 1, i + 1) = 2 * (h[i] + h[i + 1]);
        A(i + 1, i) = h[i];
        A(i, i + 1) = h[i];
    }
    A(0, 1) = 0.0;
    A(mXs.size() - 1, mXs.size() - 2) = 0.0;
    A(mXs.size() - 1, mXs.size() - 1) = 1.0;
}

void Spline::initConstantTermsVector(Eigen::VectorXf& b, std::vector<float>& h)
{
    b = Eigen::VectorXf::Zero(mXs.size());
    for (int i = 0; i < b.size() - 2; i++)
        b(i + 1) = 3.0 * (mAs[i + 2] - mAs[i + 1]) / h[i + 1] - 3.0 * (mAs[i + 1] - mAs[i]) / h[i];
}

int Spline::bisect(float aT, int aStart, int aEnd)
{
    int mid = (aStart + aEnd) / 2;
    while (!(aEnd - aStart <= 1)) {
        if (aT > mXs[mid])
            aStart = mid;
        else
            aEnd = mid;
        mid = (aStart + aEnd) / 2;
    }
    return mid;
}

std::vector<float>& Spline::getDiscretePoints()
{
    return mAs;
}

Spline2D::Spline2D(std::vector<float>& aXs, std::vector<float>& aYs, const std::vector<float>& aSpeeds)
  : mSpeeds(aSpeeds)
{
    initDistances(aXs, aYs);
    mSplineX = std::make_unique<adx::curve::Spline>(mDistances, aXs);
    mSplineY = std::make_unique<adx::curve::Spline>(mDistances, aYs);
}

Spline2D::Spline2D(std::vector<float>& aXs, std::vector<float>& aYs)
{
    initDistances(aXs, aYs);
    mSplineX = std::make_unique<adx::curve::Spline>(mDistances, aXs);
    mSplineY = std::make_unique<adx::curve::Spline>(mDistances, aYs);
}

Spline2D::Spline2D(const adx::data::Path& aPath)
{
    std::vector<float> xs, ys;

    for (auto& pose : aPath.poses) {
        xs.push_back(pose.position.x());
        ys.push_back(pose.position.y());
    }
    initDistances(xs, ys);
    mSplineX = std::make_unique<adx::curve::Spline>(mDistances, xs);
    mSplineY = std::make_unique<adx::curve::Spline>(mDistances, ys);
}

Spline2D::Spline2D(const adx::data::Plan& aPlan)
{
    std::vector<float> xs, ys;
    for (auto& point : aPlan.positions) {
        xs.push_back(point.position.x());
        ys.push_back(point.position.y());
    }

    initDistances(xs, ys);
    mSplineX = std::make_unique<adx::curve::Spline>(mDistances, xs);
    mSplineY = std::make_unique<adx::curve::Spline>(mDistances, ys);

    std::vector<float> speeds;
    for (Eigen::Vector3f speed : aPlan.speeds) {
        speeds.push_back(speed.x());
    }
    mSpeeds = speeds;
}

Spline2D::~Spline2D() {}

std::size_t Spline2D::size()
{
    return mDistances.size();
}

void Spline2D::getPosition(float aDistance, Eigen::Vector3f& aPosition)
{
    if (aDistance < 0)
        aDistance += mDistances.back();

    while (aDistance > mDistances.back())
        aDistance -= mDistances.back();

    aPosition.x() = mSplineX->eval(aDistance);
    aPosition.y() = mSplineY->eval(aDistance);
}

void Spline2D::getSpeed(const float aDistance, float& aSpeed)
{
    aSpeed =
      mSpeeds[((int)std::ceil((aDistance / mDistances.back()) * mSpeeds.size())) % mSpeeds.size()];
}

void Spline2D::getCurvature(float aDistance, float& aCurvature)
{
    if (aDistance >= mDistances[mDistances.size() - 1]) {
        aDistance = 0;
    }

    float dx = mSplineX->evalDerivative(aDistance);
    float ddx = mSplineX->evalDerivative2(aDistance);
    float dy = mSplineY->evalDerivative(aDistance);
    float ddy = mSplineY->evalDerivative2(aDistance);
    aCurvature = (ddy * dx - ddx * dy) / (dx * dx + dy * dy);
}

/*
void Spline2D::getYaw(float distance, float& yaw)
{
    float dx = mSplineX->evalDerivative(distance);
    float dy = mSplineY->evalDerivative(distance);
    yaw = std::atan2(dy, dx);
}
*/


void Spline2D::getCurveDistance(const Eigen::Vector3f& aPosition, float& aCurveDistance)
{
    std::cout << "aPosition = " << aPosition << std::endl;
    std::cout << "curveDistance = " << aCurveDistance << std::endl;
    Eigen::Vector3f s_pos;
    float s_guess = aCurveDistance;
    float dist_threshold = 2.0; // m
    getPosition(s_guess, s_pos);
    float s_opt = s_guess;

    // global search for the closest point
    float guess_dist = (aPosition - s_pos).norm();
    if (guess_dist >= dist_threshold) {
        std::vector<float> px = mSplineX->getDiscretePoints();
        std::vector<float> py = mSplineY->getDiscretePoints();

        Eigen::ArrayXf diff_x_all = Eigen::ArrayXf(px.size());
        Eigen::ArrayXf diff_y_all = Eigen::ArrayXf(py.size());

        for (unsigned int i = 0; i < px.size(); i++) {
            diff_x_all(i) = px[i];
            diff_y_all(i) = py[i];
        }

        Eigen::ArrayXf dist_square = diff_x_all.square() + diff_y_all.square();
        std::vector<float> dist_square_vec(dist_square.data(),
                                            dist_square.data() + dist_square.size());
        auto min_iter = std::min_element(dist_square_vec.begin(), dist_square_vec.end());
        s_opt = mDistances[std::distance(dist_square_vec.begin(), min_iter)];
    }

    // local newton method descent
    Eigen::Vector3f cur_pos;
    float s_old = s_opt;
    for (int i = 0; i < 20; i++) {
        getPosition(s_opt, cur_pos);
        Eigen::Vector3f ds_path(
          mSplineX->evalDerivative(s_opt), mSplineY->evalDerivative(s_opt), 0);
        Eigen::Vector3f dds_path(
          mSplineX->evalDerivative2(s_opt), mSplineY->evalDerivative2(s_opt), 0);
        Eigen::Vector3f diff = cur_pos - aPosition;
        float jac = 2.0 * diff(0) * ds_path(0) + 2.0 * diff(1) * ds_path(1);
        float hessian = 2.0 * ds_path(0) * ds_path(0) + 2.0 * diff(0) * dds_path(0) +
                         2.0 * ds_path(1) * ds_path(1) + 2.0 * diff(1) * dds_path(1);

        // descent
        s_opt -= jac / hessian;

        // Wrap s_opt
        while (s_opt > mDistances.back())
            s_opt -= mDistances.back();
        while (s_opt < mDistances.front())
            s_opt += mDistances.back();

        // stop iterating when sufficient decrease criterion is met
        if (std::abs(s_old - s_opt) <= 1e-3) {
            aCurveDistance = s_opt;
            return;
        }
        s_old = s_opt;
    }

    // if we didn't converge in MAXIT iterations we give back the initial guess
    std::cerr << "[WARNING] NEWTON METHOD REACHED MAXIT" << std::endl;
    aCurveDistance = s_opt;
}

unsigned int Spline2D::getCurveIndex(const float& distance)
{
    static unsigned int start, end, i;
    start = 0;
    end = mDistances.size() - 1;

    // binary search
    while (start != end - 1) {
        i = (start + end) / 2;

        if (mDistances[i] < distance)
            start = i;
        else
            end = i;
    }

    return start;
}

void Spline2D::initDistances(std::vector<float>& x, std::vector<float>& y)
{
    std::vector<float> ds, dx, dy;
    vectorDifference(x, dx);
    vectorDifference(y, dy);

    for (unsigned int i = 0; i < dx.size(); i++) {
        ds.push_back(std::sqrt(dx[i] * dx[i] + dy[i] * dy[i]));
    }

    mDistances = { 0 };
    cumulativeSum(ds, mDistances);
}