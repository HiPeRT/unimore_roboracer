#define _USE_MATH_DEFINES
#include <cmath>

#include "particle_filter/particle_filter.hpp"

static constexpr float FLOAT_PI = static_cast<float>(M_PI);

ParticleFilter::ParticleFilter(IRayCasting& aRayCasting)
  : mRayCasting(aRayCasting)
{}

void ParticleFilter::setConfig(std::string aConfFile)
{
    YAML::Node config = YAML::LoadFile(aConfFile);
    if (config.IsNull()) {
        std::cout << "Config is NULL." << std::endl;
        exit(-1);
    }

    mMaxParticles = config["MAX_PARTICLES"].as<int>();

    // Motion model noise
    mOdomNoise.std_yaw = config["std_yaw"].as<float>();
    mOdomNoise.std_x_mul = config["std_x_mul"].as<float>();
    mOdomNoise.std_y_mul = config["std_y_mul"].as<float>();
    mOdomNoise.std_v_mul = config["std_v_mul"].as<float>();
    mOdomNoise.std_w_mul = config["std_w_mul"].as<float>();

    // Sensor model noise
    mWeighingProfile.z_short = config["z_short"].as<float>();
    mWeighingProfile.z_max = config["z_max"].as<float>();
    mWeighingProfile.z_rand = config["z_rand"].as<float>();
    mWeighingProfile.z_hit = config["z_hit"].as<float>();
    mWeighingProfile.sigma_hit = config["sigma_hit"].as<float>();

    // parse sensor cloud descriptor
    mCloudDescriptor.maxRayIteration = config["maxRayIteration"].as<int>();
    mCloudDescriptor.maxRange = config["maxRange"].as<float>();
    mCloudDescriptor.angleMin = config["angleMin"].as<float>();
    mCloudDescriptor.angleMax = config["angleMax"].as<float>();
    mCloudDescriptor.angleIncrement = config["angleIncrement"].as<float>();
    mCloudDescriptor.angleDownsample = config["angleDownsample"].as<float>();
    mCloudDescriptor.maxRangeMeters = config["MAX_RANGE_METERS"].as<int>();
    mCloudDescriptor.nAngle =
      int((std::fabs(mCloudDescriptor.angleMax) + std::fabs(mCloudDescriptor.angleMin)) /
          mCloudDescriptor.angleIncrement);
    mRayCasting.setCloudDescriptor(mCloudDescriptor);
}

void ParticleFilter::preprocessMap(Eigen::MatrixXi& aMap)
{
    // ???
    cv::Mat img_test =
      cv::Mat::ones(cv::Size(mMapDescriptor.map_width, mMapDescriptor.map_height), CV_8UC1) * 0;
    for (int i = 0; i < mMapDescriptor.map_height; i++) {
        for (int j = 0; j < mMapDescriptor.map_width; j++) {
            if (aMap(i, j) == 0) {
                img_test.at<uchar>(i, j) = 255;
            } else {
                img_test.at<uchar>(i, j) = 0;
            }
        }
    }

    cv::Mat srcMap;
    cv::Mat distMap;
    cvtColor(img_test, img_test, cv::COLOR_GRAY2RGB);
    img_test.copyTo(srcMap);
    if (srcMap.empty()) {
        std::cout << "Could not open or find the image!" << std::endl;
        return;
    }
    cv::flip(srcMap, srcMap, -1);
    cv::flip(srcMap, srcMap, 0);
    cv::floodFill(srcMap, cv::Point(0, 0), cv::Scalar(0, 0, 0), (cv::Rect*)0, cv::Scalar(), 200);
    // Create a kernel that we will use to sharpen our image
    cv::Mat kernel = (cv::Mat_<float>(3, 3) << 1, 1, 1, 1, -8, 1, 1, 1, 1);

    cv::Mat imgLaplacian;
    cv::Mat sharp = srcMap;
    filter2D(sharp, imgLaplacian, CV_32F, kernel);
    srcMap.convertTo(sharp, CV_32F);
    cv::Mat imgResult = sharp - imgLaplacian;
    // convert back to 8bits gray scale
    imgResult.convertTo(imgResult, CV_8UC3);
    imgLaplacian.convertTo(imgLaplacian, CV_8UC3);
    // Create binary image from source image
    cv::Mat bw;
    cvtColor(srcMap, bw, cv::COLOR_BGR2GRAY);
    threshold(bw, bw, 100, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    // Perform the distance transform algorithm
    distanceTransform(bw, distMap, cv::DIST_L2, 3);
    cv::normalize(distMap, distMap, 0, 1, cv::NORM_MINMAX);

    // realloc distMap
    if (mDistanceMap != nullptr)
        delete[] mDistanceMap;
    mDistanceMap = new float[distMap.rows * distMap.cols];

    cv::Point p;
    for (int i = 0; i < distMap.rows; i++) {
        for (int j = 0; j < distMap.cols; j++) {
            p.x = j;
            p.y = i;
            mDistanceMap[i * distMap.cols + j] = distMap.at<float>(p);
        }
    }

    mRayCasting.precomputeSensorModelTable(mWeighingProfile, mMapDescriptor.MAX_RANGE_PX);
}

void ParticleFilter::spawnParticles(adx::data::Pose2f aInitialPose)
{
    mParticles.resize(mMaxParticles);
    std::normal_distribution<float> dist_x(0.0f, 0.02f);
    std::normal_distribution<float> dist_y(0.0f, 0.02f);
    for (int i = 0; i < mMaxParticles; i++) {
        mParticles[i].position.x() = aInitialPose.position.x() + dist_x(generator);
        mParticles[i].position.y() = aInitialPose.position.y() + dist_y(generator);
        mParticles[i].yaw() = aInitialPose.yaw();
        mParticles[i].weight = 1.0f;
    }
}

void ParticleFilter::resampleParticles()
{
    std::vector<adx::data::Particlefld> new_particles;
    new_particles.resize(mMaxParticles);

    double c = mParticles[0].weight;
    int i = 1;

    std::uniform_real_distribution<double> distribution(0, 1.0f / (float)mMaxParticles);
    double r = distribution(generator);

    // low variance resampling
    for (int m = 0; m < mMaxParticles; m++) {
        double U = r + float(m - 1) * 1.0f / (float)mMaxParticles;
        while (U > c) {
            i = (i + 1) % mParticles.size();
            c += mParticles[i].weight;
        }
        new_particles[m] = mParticles[i];
    }

    mParticles.swap(new_particles);
}

void ParticleFilter::applyMotionModel(float aVelocity, float aDt, float aYawRate)
{
    std::vector<adx::data::Particlefld> valid_particles;
    adx::data::Particlefld tmp_particle;
    float x, y, yaw;
    valid_particles.resize(mParticles.size());
    for (size_t i = 0; i < mParticles.size(); i++) {
        // Add odometry measurements to each particle
        if (std::fabs(aYawRate) < 0.001f) { // car going straight
            x = mParticles[i].position.x() + aVelocity * aDt * std::cos(mParticles[i].yaw());
            y = mParticles[i].position.y() + aVelocity * aDt * std::sin(mParticles[i].yaw());
            yaw = mParticles[i].yaw();
        } else { // yaw rate is not equal to zero, car turning
            x = mParticles[i].position.x() +
                (aVelocity) / aYawRate *
                  (std::sin(mParticles[i].yaw() + aYawRate * aDt) - std::sin(mParticles[i].yaw()));
            y = mParticles[i].position.y() +
                (aVelocity) / aYawRate *
                  (std::cos(mParticles[i].yaw()) - std::cos(mParticles[i].yaw() + aYawRate * aDt));
            yaw = mParticles[i].yaw() + aYawRate * aDt;
        }

        float std_x = mOdomNoise.std_x_mul + std::fabs(aVelocity * aDt * mOdomNoise.std_v_mul);
        float std_y = mOdomNoise.std_y_mul;
        // add gaussian noise
        float x_noise = 0.0f;
        float y_noise = 0.0f;
        float yaw_noise = 0.0f;

        std::normal_distribution<double> dist_x(x_noise, std_x);
        std::normal_distribution<double> dist_y(y_noise, std_y);
        std::normal_distribution<double> dist_yaw(
          yaw_noise, mOdomNoise.std_yaw + std::fabs(aYawRate * aDt * mOdomNoise.std_w_mul));
        x_noise = dist_x(generator);
        y_noise = dist_y(generator);
        double tsin = sin(yaw);
        double tcos = cos(yaw);
        double rot_x = x_noise * tcos - y_noise * tsin;
        double rot_y = x_noise * tsin + y_noise * tcos;

        tmp_particle.position.x() = x + rot_x;
        tmp_particle.position.y() = y + rot_y;

        // int c = (int)((mMapDescriptor.opp_originX - tmp_particle.position.x()) /
        //               mMapDescriptor.map_resolution);
        // int r = (int)((mMapDescriptor.opp_originY + tmp_particle.position.y()) /
        //               mMapDescriptor.map_resolution);

        // if (mDistanceMap[r * mMapDescriptor.map_width + c] >= mMapDescriptor.map_resolution) {
        tmp_particle.yaw() = yaw + dist_yaw(generator);
        valid_particles[i] = tmp_particle;
        // }
    }
    mParticles.swap(valid_particles);
    // std::cout << "[DBG] valid particles: " << mParticles.size() << std::endl;
}

void ParticleFilter::applySensorModel(std::vector<float>& aCloud, std::vector<float>& aRaysAngles)
{
    mRayCasting.weighParticles(mParticles, aCloud, mDistanceMap, mMapDescriptor, aRaysAngles);
}

void ParticleFilter::normalizeWeights()
{
    long double weightSum = 0;
    for (size_t i = 0; i < mParticles.size(); i++) {
        weightSum += mParticles[i].weight;
    }

    for (size_t i = 0; i < mParticles.size(); i++) {
        mParticles[i].weight /= weightSum;
    }
}

void ParticleFilter::updateLocalization(adx::data::Twistd& aTwist,
                                        float aDt,
                                        std::vector<float>& aDownsampledScan,
                                        std::vector<float>& aRaysAngles)
{
    applyMotionModel(aTwist.linear.x(), aDt, aTwist.angular.z());
    applySensorModel(aDownsampledScan, aRaysAngles);
    normalizeWeights();
    updatePose();

    // output estimate
    publishEstimate();
    resampleParticles();
}

void ParticleFilter::updatePose()
{
    Eigen::MatrixXd poses = Eigen::MatrixXd(mParticles.size(), 3);
    Eigen::VectorXd w = Eigen::VectorXd(mParticles.size());

    // pose estimation
    for (size_t i = 0; i < mParticles.size(); i++) {
        poses(i, 0) = mParticles[i].position.x();
        poses(i, 1) = mParticles[i].position.y();
        poses(i, 2) = mParticles[i].yaw();
        w[i] = mParticles[i].weight;
    }
    // TODO: cutoff weight outliers
    Eigen::Vector3d expected = poses.transpose() * w;
    mPose.position.x() = expected.x();
    mPose.position.y() = expected.y();
    mPose.yaw() = expected.z();

    // remap yaw to [-M_PI , M_PI]
    if (mPose.yaw() > FLOAT_PI || mPose.yaw() > -FLOAT_PI)
        mPose.yaw() -=
          static_cast<int>((mPose.yaw() + FLOAT_PI) / (2.0f * FLOAT_PI)) * (2.0f * FLOAT_PI);

    mPose.pose_covariance.setZero();
    // variance estimation (rough)
    // this can probably be done on a matrix basis
    for (size_t i = 0; i < mParticles.size(); i++) {
        mPose.pose_covariance(0, 0) +=
          mParticles[i].weight * std::pow(mParticles[i].position.x() - expected.x(), 2);
        mPose.pose_covariance(1, 1) +=
          mParticles[i].weight * std::pow(mParticles[i].position.y() - expected.y(), 2);
        mPose.pose_covariance(2, 2) +=
          mParticles[i].weight * std::pow(mParticles[i].yaw() - expected.z(), 2);
    }
}