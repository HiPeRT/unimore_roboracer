// #include <rclcpp/time.hpp>

#include "adx_data/header.hpp"

namespace adx {
namespace data {

Header& Header::operator=(Header&& aOther)
{
    if (this != &aOther) {
        timestamp = std::move(aOther.timestamp);
        frame_id = std::move(aOther.frame_id);
    }
    return *this;
}

} // namespace data
} // namespace adx
