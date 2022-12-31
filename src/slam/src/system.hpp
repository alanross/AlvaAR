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
#include "visual_frontend.hpp"

class System
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    System();

    ~System();

    void configure(int imageWidth, int imageHeight, double fx, double fy, double cx, double cy, double k1, double k2, double p1, double p2);

    void reset();

    int findCameraPose(int imageRGBADataPtr, int posePtr);

    int findPlane(int locationPtr);

    int getFramePoints(int pointsPtr);

private:
    cv::Mat processPlane(int iterations = 50);

    int processCameraPose(cv::Mat &image);

    cv::Mat ExpSO3(const cv::Mat &v);

    std::vector<Point3D> getPointCloud();

    int width;
    int height;
    int frameId_ = -1;

    std::shared_ptr<State> state_;
    std::shared_ptr<Frame> currFrame_;
    std::shared_ptr<CameraCalibration> cameraCalibration_;
    std::shared_ptr<MapManager> mapManager_;
    std::unique_ptr<Mapper> mapper_;
    std::unique_ptr<VisualFrontend> visualFrontend_;
    std::shared_ptr<FeatureExtractor> featureExtractor_;
    std::shared_ptr<FeatureTracker> featureTracker_;
};
