#include "adx_data/odometry.hpp"

namespace adx {
namespace data {

Odometry& Odometry::operator=(const Odometry& aOdometry)
{
    Header::operator=(static_cast<Header>(aOdometry));
    Pose<double, 3, false, true>::operator=(static_cast<Pose<double, 3, false, true>>(aOdometry));
    Twist<double, false, true>::operator=(static_cast<Twist<double, false, true>>(aOdometry));

    return *this;
}

Odometry& Odometry::operator=(Odometry&& aOther)
{
    Header::operator=(static_cast<Header&&>(aOther));
    Pose<double, 3, false, true>::operator=(static_cast<Pose<double, 3, false, true>&&>(aOther));
    Twist<double, false, true>::operator=(static_cast<Twist<double, false, true>&&>(aOther));

    return *this;
}

} // namespace data
} // namespace adx