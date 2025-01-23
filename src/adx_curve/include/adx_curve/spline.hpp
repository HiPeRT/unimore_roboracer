#ifndef ADX_CURVE_SPLINE_HPP
#define ADX_CURVE_SPLINE_HPP

#include <cstddef>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include "adx_data/path.hpp"
#include "adx_curve/curve.hpp"

namespace adx {
namespace curve {

/**
 * This static function implements an in-place vector difference.
 * Given a vector A it creates a new output vector B containing n-1 elements
 * where each element is defined as B(i) = A[i] - A[i-1].
 * @brief Vector difference
 * @param aInput The input vector to calculate the difference on
 * @param aVectorDifference The output vector containing the vector difference
 */
static inline void vectorDifference(std::vector<float>& aInput, std::vector<float>& aVectorDifference)
{
    for (unsigned int i = 1; i < aInput.size(); i++)
        aVectorDifference.push_back(aInput[i] - aInput[i - 1]);
}

/**
 * This static function implements a vector cumulative sum.
 * Given a vector A it creates a new output vector B of the same size
 * where each element is defined as the sum of all the preceding elements.
 * @brief Cumulative sum
 * @param aInput  The input vector to calculate the cumulative sum on
 * @param aCumSum The output vector containing the cumulative sum
 */
static inline void cumulativeSum(std::vector<float> aInput, std::vector<float>& aCumSum)
{
    float tmp = 0;
    for (unsigned int i = 0; i < aInput.size(); i++) {
        tmp += aInput[i];
        aCumSum.push_back(tmp);
    }
}

/**
 * @brief This class implements a 1D cubic spline.
 *
 */
class Spline
{
  public:
    /**
     * Create a new Spline that interpolates the points passed as parameters.
     * Prints a warning if the input vectors do not have the same size.
     * @brief Constructor
     * @param x The vector containing the x axis values for each point
     * @param y The vector containing the y axis values for each point
     */
    Spline(std::vector<float>& aXs, std::vector<float>& aYs);

    /**
     * This function evaluates the interpolated spline in a certain point.
     * @brief Evaluates the spline
     * @param t The point on the x axis
     * @return The value of the spline in t
     */
    float eval(float t);

    /**
     * This function evaluates the interpolated spline's derivative
     * in a certain point.
     * @brief Evaluates the spline's derivative
     * @param t The point on the x axis
     * @return The value of the spline's derivative in t
     */
    float evalDerivative(float t);

    /**
     * This function evaluates the interpolated spline's second derivative
     * in a certain point.
     * @brief Evaluates the spline's second derivative
     * @param t The point on the x axis
     * @return The value of the spline's second derivative in t
     */
    float evalDerivative2(float t);

    /**
     * Returns the discrete array of the spline's x axis
     * @return the spline's x axis
     */
    std::vector<float>& getDiscretePoints();


  private:
    /**
     * Vector of x-axis coordinates
     */
    std::vector<float> mXs;

    /**
     * Spline coefficient vectors for each segment
     */
    std::vector<float> mAs, mBs, mCs, mDs;

    /**
     * Initializes the coefficients matrix A based on the
     * supplied difference vector h.
     * @brief Initialize coefficients matrix
     * @param A The output coefficients matrix
     * @param h The input difference vector
     */
    void initCoefficientsMatrix(Eigen::MatrixXf& A, std::vector<float>& h);

    /**
     * Initializes the constant terms vector b based on the
     * supplied difference vector h.
     * @brief Initialize the constant terms vector
     * @param b The output constant terms vector
     * @param h The input difference vector
     */
    void initConstantTermsVector(Eigen::VectorXf& b, std::vector<float>& h);

    /**
     * Implements the bisection search on the spline
     * @brief Bisection method
     * @param t     The t to bisect
     * @param start The start of the range
     * @param end   The end of the range
     * @return The segment where the x axis is closest to t
     */
    int bisect(float t, int start, int end);

};

/**
 * @brief This class implements a 2D cubic spline.
 */
class Spline2D : public Curve
{
  public:
    /**
     * Create a new Spline2D that interpolates the points passed as parameters.
     * @brief Constructor
     * @param aXs The vector containing the x axis values for each point
     * @param aYs The vector containing the y axis values for each point
     * @param aSpeeds The desired speed for each interpolation point
     */
    Spline2D(std::vector<float>& aXs, std::vector<float>& aYs, const std::vector<float>& aSpeeds);

    /**
     * Create a new Spline2D that interpolates the points passed as parameters.
     * @brief Constructor
     * @param aXs The vector containing the x axis values for each point
     * @param aYs The vector containing the y axis values for each point
     */
    Spline2D(std::vector<float>& aXs, std::vector<float>& aYs);

    /**
     * Create a new Spline2D that interpolates the plan passed as parameter.
     * @brief Constructor
     * @param aPlan A ros message containing a vector of 3D points plus 3D velocities.
     */
    Spline2D(const adx::data::Plan& aPlan);

    /**
     * Create a new Spline2D that interpolates the path passed as parameter.
     * @brief Constructor
     * @param aPath A ros message containing a vector of 3D points.
     */
    Spline2D(const adx::data::Path& aPath);

    ~Spline2D();

    /**
     * @return unsigned int - the size of the distance vector
     */
    std::size_t size();

    /**
     * Outputs the position on the spline given a certain spline distance
     * @brief Outputs the position
     * @param aDistance The distance from the spline starting point
     * @param aPosition The output x, y point on the interpolated spline
     */
    void getPosition(float aDistance, Eigen::Vector3f& aPosition);

    /**
     * Outputs the desired speed based on the supplied spline distance
     * @brief Outputs the speed
     * @param aDistance The distance from the spline starting point
     * @param aSpeed    The output desired speed
     */
    void getSpeed(const float aDistance, float& aSpeed);

    /**
     * Outputs the curvature of the spline based on the current distance
     * @brief Outputs the curvature
     * @param aDistance  The distance from the spline starting point
     * @param aCurvature The output curvature
     */
    void getCurvature(float aDistance, float& aCurvature);

    /**
     * Outputs the yaw angle on the supplied spline distance
     * @brief Outputs the Yaw
     * @param aDistance The distance from the spline starting point
     * @param aYaw      The output yaw angle
     */
    // void getYaw(float distance, float& yaw);

    /**
     * Outputs the closest point on the curve (as a distance from the spline
     * starting point) given a certain (x, y) position
     * @brief Closest curve distance
     * @param aPosition The input point
     * @param aDistance The ourput curve distance
     */
    void getCurveDistance(const Eigen::Vector3f& aPosition, float& aDistance);

    /**
     * Outputs the index of the closest point on the curve given a certain (x, y) position
     * @brief Get the curve Index for that distance
     * @param distance the distance at which the index is wanted
     * @return unsigned int
     */
    unsigned int getCurveIndex(const float& aDistance);

  private:
    /**
     * The two splines representing the spline value for each dimension
     */
    std::unique_ptr<adx::curve::Spline> mSplineX, mSplineY;

    /**
     * Spline distances used as x values for each spline
     */
    std::vector<float> mDistances;

    /**
     * Desired speeds vector for each point in the spline
     */
    std::vector<float> mSpeeds;

    /**
     * Initializes the distances vector given the points to interpolate
     * @brief Initializes the distances vector
     * @param x The x-axis points vector
     * @param y The y-axis points vector
     */
    void initDistances(std::vector<float>& aXs, std::vector<float>& aYs);
};

}} // namespace adx::curve

#endif // ADX_CURVE_SPLINE_HPP