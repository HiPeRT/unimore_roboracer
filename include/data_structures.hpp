#ifndef __DATA_STRUCTURES_HPP__
#define __DATA_STRUCTURES_HPP__

struct Map_t
{
    int MAX_RANGE_PX;
    float opp_originX;
    float opp_originY;
    float map_resolution;
    float map_originX;
    float map_originY;
    int map_height;
    int map_width;
};

struct Cloud_t
{
    int maxRayIteration;
    float maxRange;
    float angleMin;
    float angleMax;
    float angleIncrement;
    float angleDownsample;
    float maxRangeMeters;
    int nAngle;
};

struct WeighingProfile_t
{
    float z_hit;
    float sigma_hit;
    float z_short;
    float z_max;
    float z_rand;
};

struct MotionModelNoise_t
{
    float std_w_mul;
    float std_yaw;
    float std_x_mul;
    float std_y_mul;
    float std_v_mul;
};

enum class Implementation : unsigned int
{
    CPU = 0,
    GPU = 1,
    FPGA = 2
};

#endif //__DATA_STRUCTURES_HPP__
