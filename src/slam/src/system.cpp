#include "system.hpp"
#include <memory>
#include <iostream>

System::System()
{
    width_ = 0;
    height_ = 0;
}

System::~System()
{
}

void System::configure(int imageWidth, int imageHeight, double fx, double fy, double cx, double cy, double k1, double k2, double p1, double p2)
{
    width_ = imageWidth;
    height_ = imageHeight;

    state_ = std::make_shared<State>(imageWidth, imageHeight);
    cameraCalibration_ = std::make_shared<CameraCalibration>(fx, fy, cx, cy, k1, k2, p1, p2, imageWidth, imageHeight, 20);

    currFrame_ = std::make_shared<Frame>(cameraCalibration_, state_->frameMaxCellSize_);

    featureExtractor_ = std::make_shared<FeatureExtractor>(state_->extractorMaxQuality_);
    featureTracker_ = std::make_shared<FeatureTracker>(state_->trackerMaxIterations_, state_->trackerMaxPxPrecision_);

    mapManager_ = std::make_shared<MapManager>(state_, currFrame_, featureExtractor_);
    mapper_ = std::make_unique<Mapper>(state_, mapManager_, currFrame_);
    visualFrontend_ = std::make_unique<VisualFrontend>(state_, currFrame_, mapManager_, featureTracker_);
}

void System::reset()
{
    std::cout << "- [System]: Reset" << std::endl;

    currFrame_->reset();
    visualFrontend_->reset();
    mapManager_->reset();
    state_->reset();

    frameId_ = -1;
}

int System::findCameraPoseWithIMU(int imageRGBADataPtr, int imuDataPtr, int posePtr)
{
    auto *frameData = reinterpret_cast<uint8_t *>(imageRGBADataPtr);
    auto *imuData = reinterpret_cast<double *>(imuDataPtr);
    auto *poseData = reinterpret_cast<float *>(posePtr);

    cv::Mat frame = cv::Mat(height_, width_, CV_8UC4, frameData);
    cv::cvtColor(frame, frame, cv::COLOR_RGBA2GRAY);

    //quaternion in format wxyz. -x to mirror slam format
    Eigen::Quaterniond orientation(imuData[0], -imuData[1], imuData[2], imuData[3]);
    Eigen::Matrix3d rwc = orientation.toRotationMatrix().inverse();
    Eigen::Vector3d twc(0.0, 0.0, 0.0);

    int motionSampleSize = 7;
    int motionSampleCount = (int) imuData[4] * motionSampleSize;

    for (int i = 5; i < motionSampleCount; i += motionSampleSize)
    {
        // { timestamp, gx, gy, gz, ax, ay, az }
        auto timestamp = (uint64_t) imuData[i];
        Eigen::Vector3d gyr(imuData[i + 1], imuData[i + 2], imuData[i + 3]);
        Eigen::Vector3d acc(imuData[i + 4], imuData[i + 5], imuData[i + 6]);
    }

    Utils::toPoseArray(rwc, twc, poseData);

    return 1;
}

int System::findCameraPose(int imageRGBADataPtr, int posePtr)
{
    auto *frameData = reinterpret_cast<uint8_t *>(imageRGBADataPtr);
    auto *poseData = reinterpret_cast<float *>(posePtr);

    cv::Mat frame = cv::Mat(height_, width_, CV_8UC4, frameData);
    cv::cvtColor(frame, frame, cv::COLOR_RGBA2GRAY);

    int status = processCameraPose(frame);

    Eigen::Vector3d twc = currFrame_->getTwc().translation();
    Eigen::Matrix3d rwc = currFrame_->getTwc().rotationMatrix();

    Utils::toPoseArray(rwc, twc, poseData);

    return status;
}

int System::findPlane(int locationPtr)
{
    cv::Mat mat = processPlane(50);

    if (mat.empty())
    {
        return 0;
    }

    auto *poseData = reinterpret_cast<float *>(locationPtr);

    Utils::toPoseArray(mat, poseData);

    return 1;
}

int System::getFramePoints(int pointsPtr)
{
    auto *data = reinterpret_cast<int *>(pointsPtr);

    int numPoints = currFrame_->getKeypoints2d().size();
    int n = std::min(numPoints * 2, 4096);

    for (int i = 0, j = 0; i < n; ++i)
    {
        cv::Point2f p = currFrame_->getKeypoints2d()[i].unpx_;
        data[j++] = (int) p.x;
        data[j++] = (int) p.y;
    }

    return numPoints;
}

int System::processCameraPose(cv::Mat &image)
{
    frameId_++;

    uint64_t timeStamp = duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    currFrame_->updateFrame(frameId_, timeStamp);

    bool isKeyFrameRequired = visualFrontend_->visualTracking(image, timeStamp);

    if (state_->slamResetRequested_)
    {
        reset();
        return 2;
    }

    if (!state_->slamReadyForInit_)
    {
        return 3;
    }

    if (isKeyFrameRequired)
    {
        Keyframe kf(currFrame_->keyframeId_, image);
        mapper_->addNewKeyframe(kf);
    }

    return 1;
}

cv::Mat System::processPlane(int iterations)
{
    cv::Mat plane;

    std::vector<Eigen::Vector3d> mapPoints = mapManager_->getCurrentFrameMapPoints();

    const long numMapPointsAll = mapPoints.size();

    if (numMapPointsAll < 16)
    {
        std::cout << "- [System]: Too few points to detect plane: " << numMapPointsAll << std::endl;

        return plane; //empty plane
    }

    std::vector<cv::Mat> points(numMapPointsAll);
    std::vector<int> indices(numMapPointsAll);
    std::vector<float> distances;
    float bestDist = 1e10;

    for (int i = 0; i < numMapPointsAll; i++)
    {
        cv::Mat pointWorldPos(1, 3, CV_32F);
        cv::eigen2cv(mapPoints[i], pointWorldPos);
        points[i] = pointWorldPos;
        indices[i] = i;
    }

    //RANSAC to find inliers
    for (int n = 0; n < iterations; n++)
    {
        // Pick 3 random points
        int numToPick = 3;
        std::vector<int> indicesPicked;
        std::sample(indices.begin(), indices.end(), std::back_inserter(indicesPicked), numToPick, std::mt19937{std::random_device{}()});

        cv::Mat u, w, vt;
        cv::Mat A(3, 4, CV_32F);
        A.col(3) = cv::Mat::ones(3, 1, CV_32F);

        for (int i = 0; i < numToPick; i++)
        {
            A.row(i).colRange(0, 3) = points[indicesPicked[i]].t();
        }

        cv::SVDecomp(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

        const float a = vt.at<float>(3, 0);
        const float b = vt.at<float>(3, 1);
        const float c = vt.at<float>(3, 2);
        const float d = vt.at<float>(3, 3);

        std::vector<float> dists(numMapPointsAll, 0);

        const float f = 1.0f / std::sqrt(a * a + b * b + c * c + d * d);

        for (int i = 0; i < numMapPointsAll; i++)
        {
            dists[i] = std::fabs(points[i].at<float>(0) * a + points[i].at<float>(1) * b + points[i].at<float>(2) * c + d) * f;
        }

        std::vector<float> distsSorted = dists;
        sort(distsSorted.begin(), distsSorted.end());

        int nth = std::max((int) (0.2 * numMapPointsAll), 20);
        const float medianDist = distsSorted[nth];

        if (medianDist < bestDist)
        {
            bestDist = medianDist;
            distances = dists;
        }
    }

    // Compute threshold inlier/outlier
    const float threshold = 1.4 * bestDist;
    std::vector<cv::Mat> pointsInliers;

    for (int i = 0; i < numMapPointsAll; i++)
    {
        if (distances[i] < threshold)
        {
            pointsInliers.push_back(points[i]);
        }
    }

    // convert cam pos from Eigen to CV Mat format
    Eigen::Vector3d twc = currFrame_->getTwc().translation();
    Eigen::Matrix3d rwc = currFrame_->getTwc().rotationMatrix();
    cv::Mat camPose(4, 4, CV_32F);

    Utils::toPoseMat(rwc, twc, camPose);

    // arbitrary orientation along normal
    float rang = -3.14f / 2 + ((float) rand() / (float) RAND_MAX) * 3.14f;

    const long numInliers = pointsInliers.size();

    // recompute plane with all pointsInliers
    cv::Mat A = cv::Mat(numInliers, 4, CV_32F);
    A.col(3) = cv::Mat::ones(numInliers, 1, CV_32F);

    cv::Mat origin = cv::Mat::zeros(3, 1, CV_32F);

    for (int i = 0; i < numInliers; i++)
    {
        cv::Mat Xw = pointsInliers[i];
        origin += Xw;
        A.row(i).colRange(0, 3) = Xw.t();
    }

    A.resize(numInliers);

    cv::Mat u, w, vt;
    cv::SVDecomp(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    float a = vt.at<float>(3, 0);
    float b = vt.at<float>(3, 1);
    float c = vt.at<float>(3, 2);

    origin = origin * (1.0f / numInliers);
    const float f = 1.0f / sqrt(a * a + b * b + c * c);

    cv::Mat Oc = -camPose.colRange(0, 3).rowRange(0, 3).t() * camPose.rowRange(0, 3).col(3);
    cv::Mat mXC = Oc - origin;

    if ((mXC.at<float>(0) * a + mXC.at<float>(1) * b + mXC.at<float>(2) * c) > 0)
    {
        a = -a;
        b = -b;
        c = -c;
    }

    const float nx = a * f;
    const float ny = b * f;
    const float nz = c * f;

    cv::Mat normal = (cv::Mat_<float>(3, 1) << nx, ny, nz);
    cv::Mat up = (cv::Mat_<float>(3, 1) << 0.0f, 1.0f, 0.0f);
    cv::Mat v = up.cross(normal);
    const float sa = cv::norm(v);
    const float ca = up.dot(normal);
    const float ang = atan2(sa, ca);

    plane = cv::Mat::eye(4, 4, CV_32F);

    plane.rowRange(0, 3).colRange(0, 3) = Utils::expSO3(v * ang / sa) * Utils::expSO3(up * rang);
    origin.copyTo(plane.col(3).rowRange(0, 3));

    return plane;
}
