#include "system.hpp"
#include <memory>
#include <iostream>

System::System()
{
}

System::~System()
{
}

void System::configure(int imageWidth, int imageHeight, double fx, double fy, double cx, double cy, double k1, double k2, double p1, double p2)
{
    width = imageWidth;
    height = imageHeight;

    state_ = std::make_shared<State>(imageWidth, imageHeight);
    cameraCalibration_ = std::make_shared<CameraCalibration>(fx, fy, cx, cy, k1, k2, p1, p2, imageWidth, imageHeight);

    currFrame_ = std::make_shared<Frame>(cameraCalibration_, state_->frameMaxCellSize_);
    featureExtractor_ = std::make_shared<FeatureExtractor>(state_->extractorMaxQuality_);
    featureTracker_ = std::make_shared<FeatureTracker>(state_->trackerMaxIterations_, state_->trackerMaxPxPrecision_);
    mapManager_ = std::make_shared<MapManager>(state_, currFrame_, featureExtractor_);
    visualFrontend_ = std::make_unique<VisualFrontend>(state_, currFrame_, mapManager_, featureTracker_);
    mapper_ = std::make_unique<Mapper>(state_, mapManager_, currFrame_);

    time = 1;
}

void System::reset()
{
    std::cout << "- [System]: Reset" << std::endl;

    currFrame_->reset();
    visualFrontend_->reset();
    mapManager_->reset();
    mapper_->reset();
    state_->reset();

    frameId_ = -1;
}

int System::findCameraPose(int imageRGBADataPtr, int posePtr)
{
    auto *frameData = reinterpret_cast<uint8_t *>(imageRGBADataPtr);
    auto *poseData = reinterpret_cast<float *>(posePtr);

    cv::Mat frame = cv::Mat(height, width, CV_8UC4, frameData);
    cv::cvtColor(frame, frame, cv::COLOR_RGBA2GRAY);

    int status = processCameraPose(frame);

    Eigen::Vector3d twc = currFrame_->getTwc().translation();
    Eigen::Matrix3d rwc = currFrame_->getTwc().rotationMatrix();

    poseData[0] = rwc(0, 0);
    poseData[1] = rwc(0, 1);
    poseData[2] = rwc(0, 2);
    poseData[3] = 0.0;

    poseData[4] = rwc(1, 0);
    poseData[5] = rwc(1, 1);
    poseData[6] = rwc(1, 2);
    poseData[7] = 0.0;

    poseData[8] = rwc(2, 0);
    poseData[9] = rwc(2, 1);
    poseData[10] = rwc(2, 2);
    poseData[11] = 0.0;

    poseData[12] = twc.x();
    poseData[13] = twc.y();
    poseData[14] = twc.z();
    poseData[15] = 1.0;

    return status;
}

int System::findPlane(int locationPtr)
{
    cv::Mat m = processPlane(50);

    if (m.empty())
    {
        return 0;
    }

    auto *poseData = reinterpret_cast<float *>(locationPtr);

    poseData[0] = m.at<float>(0, 0);
    poseData[1] = m.at<float>(1, 0);
    poseData[2] = m.at<float>(2, 0);
    poseData[3] = 0.0;

    poseData[4] = m.at<float>(0, 1);
    poseData[5] = m.at<float>(1, 1);
    poseData[6] = m.at<float>(2, 1);
    poseData[7] = 0.0;

    poseData[8] = m.at<float>(0, 2);
    poseData[9] = m.at<float>(1, 2);
    poseData[10] = m.at<float>(2, 2);
    poseData[11] = 0.0;

    poseData[12] = m.at<float>(0, 3);
    poseData[13] = m.at<float>(1, 3);
    poseData[14] = m.at<float>(2, 3);
    poseData[15] = 1.0;

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

    currFrame_->updateFrame(frameId_, time);

    bool isKeyFrameRequired = visualFrontend_->visualTracking(image, time);

    time++;

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
    camPose.at<float>(0, 0) = rwc(0, 0);
    camPose.at<float>(0, 1) = rwc(1, 0);
    camPose.at<float>(0, 2) = rwc(2, 0);
    camPose.at<float>(0, 3) = twc.x();

    camPose.at<float>(1, 0) = rwc(0, 1);
    camPose.at<float>(1, 1) = rwc(1, 1);
    camPose.at<float>(1, 2) = rwc(2, 1);
    camPose.at<float>(1, 3) = twc.y();

    camPose.at<float>(2, 0) = rwc(0, 2);
    camPose.at<float>(2, 1) = rwc(1, 2);
    camPose.at<float>(2, 2) = rwc(2, 2);
    camPose.at<float>(2, 3) = twc.z();

    camPose.at<float>(3, 0) = 0.0;
    camPose.at<float>(3, 1) = 0.0;
    camPose.at<float>(3, 2) = 0.0;
    camPose.at<float>(3, 3) = 1.0;

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

    plane.rowRange(0, 3).colRange(0, 3) = ExpSO3(v * ang / sa) * ExpSO3(up * rang);
    origin.copyTo(plane.col(3).rowRange(0, 3));

    return plane;
}

std::vector<Point3D> System::getPointCloud()
{
    return mapManager_->pointCloud_;
}

cv::Mat System::ExpSO3(const cv::Mat &v)
{
    const float x = v.at<float>(0);
    const float y = v.at<float>(1);
    const float z = v.at<float>(2);
    const float eps = 1e-4;

    cv::Mat I = cv::Mat::eye(3, 3, CV_32F);

    const float d2 = x * x + y * y + z * z;
    const float d = sqrt(d2);

    cv::Mat W = (cv::Mat_<float>(3, 3) << 0, -z, y, z, 0, -x, -y, x, 0);

    if (d < eps)
    {
        return (I + W + 0.5f * W * W);
    }
    else
    {
        return (I + W * sin(d) / d + W * W * (1.0f - cos(d)) / d2);
    }
}