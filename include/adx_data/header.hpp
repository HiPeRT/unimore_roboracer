#ifndef ADX_DATA_HEADER_HPP
#define ADX_DATA_HEADER_HPP

#include <Eigen/Core>

#include "adx_data/timestamp.hpp"

namespace adx {
namespace data {

/**
 * @brief Generic data header with timestamp and frame_id
 *
 * This header stores a timestamp of the received data and a frame_id to identify in which set of
 * coordinates a specific message resides
 */
struct Header
{
    /**
     * @see adx::data::TimeStamp
     */
    adx::data::TimeStamp timestamp;

    /**
     * @brief Frame identifier
     *
     * An identifier to the frame of reference for the different sets of coordinates
     *
     * TODO: use a static string, possibly implement a class for it
     */
    std::string frame_id;

    /**
     * @brief Construct a new Header object
     *
     * Default constructor, this should just zero-initialize the matrix.
     */
    Header() = default;

    /**
     * @brief Copy-constructor
     *
     * @param aHeader a Header with values to move
     * @return Header& reference to the initialized Header object
     */
    // constexpr Header(const Header& aHeader) { *this = operator=(aHeader); }
    Header(const Header& aHeader) { *this = operator=(aHeader); }

    /**
     * @brief Copy-assignment operator
     *
     * @param aHeader a Header with values to copy
     * @return Header& reference to the initialized Header object
     */
    // constexpr Header& operator=(const Header& aHeader)
    Header& operator=(const Header& aHeader)
    {
        timestamp = aHeader.timestamp;
        frame_id = aHeader.frame_id;

        return *this;
    }

    /**
     * @brief Move-assignment operator
     *
     * @param aOther a Header with values to move
     * @return Header& reference to the initialized Header object
     */
    Header& operator=(Header&& aOther);
};

} // nmespace data
} // namespace adx

#endif // ADX_DATA_HEADER_HPP
