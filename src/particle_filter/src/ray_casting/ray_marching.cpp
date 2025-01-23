#include "ray_casting/ray_marching.hpp"

RayMarching::RayMarching() {}

void RayMarching::castRays(std::vector<adx::data::Particlefld>& aParticles,
                           float* aDistMap,
                           Map_t& aMapData,
                           std::vector<float>& aRaysAngles,
                           float* aRays)
{
    int numParticles = aParticles.size();
    int numRays = aRaysAngles.size();

    for (int i = 0; i < numParticles; ++i) {
        for (int j = 0; j < numRays; ++j) {
            float angle = (aParticles[i].yaw() + mCloudData.angleMin) + aRaysAngles[j];
            float rayPoseX = aParticles[i].position.x();
            float rayPoseY = aParticles[i].position.y();
            float t = 0.0f;
            float out = mCloudData.maxRange;

            // march current ray
            while (t < mCloudData.maxRayIteration) {
                int c = (int)((aMapData.opp_originX - rayPoseX) / aMapData.map_resolution);
                int r = (int)((aMapData.opp_originY + rayPoseY) / aMapData.map_resolution);

                if (c < 0 || c >= aMapData.map_width || r < 0 || r > aMapData.map_height) {
                    out = mCloudData.maxRange;
                    break;
                }

                double distance = aDistMap[r * aMapData.map_width + c];
                rayPoseX += distance * std::cos(angle);
                rayPoseY += distance * std::sin(angle);

                if (distance <= aMapData.map_resolution) {
                    float xd = rayPoseX - aParticles[i].position.x();
                    float yd = rayPoseY - aParticles[i].position.y();
                    out = sqrtf(xd * xd + yd * yd);
                    break;
                }

                t += fmaxf(distance * 0.999f, 1.0);
            }

            // save casted ray
            aRays[i * numRays + j] = out;
        }
    }
}