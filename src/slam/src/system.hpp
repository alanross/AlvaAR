#pragma once

#include <memory>
#include <vector>
#include <queue>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <Eigen/Core>
#include "camera_calibration.hpp"
#include "feature_extractor.hpp"
#include "feature_tracker.hpp"
#include "frame.hpp"
#include "mapper.hpp"
#include "map_manager.hpp"
#include "state.hpp"
#include "utils.hpp"
#include "visual_frontend.hpp"

class System
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    System();

    ~System();

    void configure(int imageWidth, int imageHeight, double fx, double fy, double cx, double cy, double k1, double k2, double p1, double p2);

    void reset();

    int findCameraPoseWithIMU(int imageRGBADataPtr, int imuDataPtr, int posePtr);

    int findCameraPose(int imageRGBADataPtr, int posePtr);

    int findPlane(int locationPtr, int numIterations);

    int getFramePoints(int pointsPtr);

private:
    cv::Mat processPlane(std::vector<Eigen::Vector3d> mapPoints, Sophus::SE3d Twc, int numIterations = 50);

    int processCameraPose(cv::Mat &image, double timestamp);

    std::shared_ptr<State> state_;
    std::shared_ptr<Frame> currFrame_;
    std::shared_ptr<CameraCalibration> cameraCalibration_;
    std::shared_ptr<MapManager> mapManager_;
    std::shared_ptr<Mapper> mapper_;
    std::unique_ptr<VisualFrontend> visualFrontend_;
    std::shared_ptr<FeatureExtractor> featureExtractor_;
    std::shared_ptr<FeatureTracker> featureTracker_;

    Eigen::Vector3d currTranslation_;
    Eigen::Vector3d prevTranslation_;
};
