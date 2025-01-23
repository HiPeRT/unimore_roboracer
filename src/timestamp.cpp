#include <iostream>

#include "adx_data/timestamp.hpp"

namespace adx {
namespace data {

TimeStamp& TimeStamp::operator=(const std::chrono::steady_clock::time_point& aTimePoint)
{
    timestamp = aTimePoint;

    return *this;
}

TimeStamp& TimeStamp::operator=(TimeStamp&& aOther)
{
    timestamp = std::move(aOther.timestamp);

    return *this;
}

bool TimeStamp::operator<(const TimeStamp& rhs) const
{
    return timestamp < rhs.timestamp;
}

bool TimeStamp::operator<=(const TimeStamp& rhs) const
{
    return timestamp <= rhs.timestamp;
}

bool TimeStamp::operator>(const TimeStamp& rhs) const
{
    return timestamp > rhs.timestamp;
}

bool TimeStamp::operator>=(const TimeStamp& rhs) const
{
    return timestamp >= rhs.timestamp;
}

bool TimeStamp::operator==(const TimeStamp& rhs) const
{

    return timestamp == rhs.timestamp;
}

bool TimeStamp::operator!=(const TimeStamp& rhs) const
{
    return timestamp != rhs.timestamp;
}

std::ostream& operator<<(std::ostream& os, const TimeStamp& rhs)
{
    os << std::chrono::time_point_cast<std::chrono::nanoseconds>(rhs.timestamp)
            .time_since_epoch()
            .count();
    return os;
}

std::ofstream& operator<<(std::ofstream& ofs, const TimeStamp& rhs)
{
    ofs << std::chrono::time_point_cast<std::chrono::nanoseconds>(rhs.timestamp)
             .time_since_epoch()
             .count();
    return ofs;
}

} // namespace data
} // namespace adx
