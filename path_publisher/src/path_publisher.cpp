#include <fstream>
#include <iostream>
#include <string>

#include "path_publisher/path_publisher.hpp"

PathPublisher::PathPublisher()
{
    mInitialized = false;
}

PathPublisher::~PathPublisher(){}

void PathPublisher::openFile()
{
    std::cout << "opening " << mFilename << std::endl;

    mTrjFile.open(mFilename, std::ifstream::out);
    if(!mTrjFile.good()) {
        std::cerr << mFilename << ": bad file" << std::endl;
        exit(-1);
    }
}

void PathPublisher::closeFile()
{
    mTrjFile.close();
}


void PathPublisher::read_plan()
{
    adx::data::Point3f cur_point;
    adx::data::Vector3f cur_speed;
    std::string line;

    openFile();

    // remove header
    std::getline(mTrjFile, line);

    // array for a cleaner loop
    float *const ar_point[6] = {
        &cur_point.position.x(),
        &cur_point.position.y(),
        &cur_point.position.z(),
        &cur_speed.x(),
        &cur_speed.y(),
        &cur_speed.z() };

    while(std::getline(mTrjFile, line))
    {
        int start_pos = 0, end_pos;
        for (int i = 0; i < 6; ++i)
        {
            end_pos = line.find(',', start_pos);
            *ar_point[i] = std::stod(line.substr(start_pos, end_pos - start_pos));
            start_pos = end_pos + 1;
        }

        mPlan.positions.push_back(cur_point);
        mPlan.speeds.push_back(cur_speed);
    }
    closeFile();

    std::cout << "Read " << mPlan.positions.size() << " points." << std::endl;
}
