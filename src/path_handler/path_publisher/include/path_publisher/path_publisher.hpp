#ifndef PATH_PUBLISHER_PATH_PUBLISHER_HPP
#define PATH_PUBLISHER_PATH_PUBLISHER_HPP

#include <fstream>
#include <vector>

#include <adx_data/point.hpp>
#include <adx_data/path.hpp>

enum class PATH_MODE : unsigned int
{
    PLAN    = (1u << 0),
    PATH    = (1u << 1),
    BIN     = (1u << 2)
};

inline constexpr unsigned int
operator&(PATH_MODE x, PATH_MODE y)
{
    return (static_cast<unsigned int>(x) & static_cast<unsigned int>(y));
}

class PathPublisher
{
public:
    PathPublisher();
    ~PathPublisher();

    virtual void publish() = 0;

protected:
    bool mInitialized;

    std::string mFilename;
    std::ifstream mTrjFile;

    adx::data::Plan mPlan;

    void openFile();
    void closeFile();
    void read_plan();
};

#endif // PATH_PUBLISHER_PATH_PUBLISHER_HPP
