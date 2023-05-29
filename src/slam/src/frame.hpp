#pragma once

#include <vector>
#include <set>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>
#include "camera_calibration.hpp"

struct Keypoint
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    int keypointId_;
    cv::Point2f px_;
    cv::Point2f unpx_;
    Eigen::Vector3d bv_;
    cv::Mat desc_;
    bool is3d_;

    Keypoint() : keypointId_(-1), is3d_(false)
    {}

    // For using kps in ordered containers
    bool operator<(const Keypoint &kp) const
    {
        return keypointId_ < kp.keypointId_;
    }
};

class Frame
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Frame();

    Frame(std::shared_ptr<CameraCalibration> cameraCalibration, const size_t cellSize);

    Frame(const Frame &frame);

    std::vector<Keypoint> getKeypoints() const;

    std::vector<Keypoint> getKeypoints2d() const;

    std::vector<Keypoint> getKeypoints3d() const;

    // Return vector of keypoints raw pixel positions
    std::vector<cv::Point2f> getKeypointsPx() const;

    // Return vector of keypoints undistorted pixel positions
    std::vector<cv::Point2f> getKeypointsUnPx() const;

    Keypoint getKeypointById(const int keypointId) const;

    // Compute keypoint from raw pixel position
    void computeKeypoint(const cv::Point2f &point, Keypoint &keypoint);

    // Create keypoint from raw pixel position
    Keypoint computeKeypoint(const cv::Point2f &point, const int keypointId);

    // Add keypoint object to vector of kps
    void addKeypoint(const Keypoint &keypoint);

    // Add new keypoint from raw pixel position
    void addKeypoint(const cv::Point2f &pt, const int keypointId);

    void addKeypoint(const cv::Point2f &point, const int keypointId, const cv::Mat &descriptor);

    void updateKeypoint(const cv::Point2f &point, Keypoint &keypoint);

    void updateKeypoint(const int keypointId, const cv::Point2f &point);

    void updateKeypointDesc(const int keypointId, const cv::Mat &descriptor);

    bool updateKeypointId(const int prevKeypointId, const int newKeypointId, const bool is3d);

    void removeKeypoint(const Keypoint &keypoint);

    void removeKeypointById(const int keypointId);

    void addKeypointToGrid(const Keypoint &keypoint);

    void removeKeypointFromGrid(const Keypoint &keypoint);

    void updateKeypointInGrid(const Keypoint &prevKeypoint, const Keypoint &newKeypoint);

    int getKeypointCellIdx(const cv::Point2f &point) const;

    std::vector<Keypoint> getSurroundingKeypoints(const cv::Point2f &point) const;

    void turnKeypoint3d(const int keypointId);

    bool isObservingKeypoint(const int keypointId) const;

    Sophus::SE3d getTcw() const;

    Sophus::SE3d getTwc() const;

    Eigen::Matrix3d getRcw() const;

    Eigen::Matrix3d getRwc() const;

    void setTwc(const Sophus::SE3d &Twc);

    void setTwc(const Eigen::Matrix3d &Rwc, Eigen::Vector3d &twc);

    std::map<int, int> getCovisibleKeyframeMap() const;

    void addCovisibleKeyframe(const int keyframeId);

    void removeCovisibleKeyframe(const int keyframeId);

    void decreaseCovisibleKeyframe(const int keyframeId);

    cv::Point2f projCamToImageDist(const Eigen::Vector3d &point) const;

    cv::Point2f projCamToImage(const Eigen::Vector3d &point) const;

    cv::Point2f projDistCamToImage(const Eigen::Vector3d &point) const;

    Eigen::Vector3d projCamToWorld(const Eigen::Vector3d &point) const;

    Eigen::Vector3d projWorldToCam(const Eigen::Vector3d &point) const;

    cv::Point2f projWorldToImage(const Eigen::Vector3d &point) const;

    cv::Point2f projWorldToImageDist(const Eigen::Vector3d &point) const;

    bool isInImage(const cv::Point2f &point) const;

    void reset();

    // For using frame in ordered containers
    bool operator<(const Frame &f) const
    {
        return id_ < f.id_;
    }

    // Frame info
    int id_;
    int keyframeId_;
    double timestamp_;

    // Map of observed keypoints
    std::unordered_map<int, Keypoint> mapKeypoints_;

    // Grid of keypoint sorted by cell numbers and scale (We use const pointer to reference the keypoints in vkps_
    // hence we should only use the grid to read keypoints)
    std::vector<std::vector<int>> gridKeypointsIds_;
    size_t gridCells_;
    size_t numOccupiedCells_;
    size_t cellSize_;
    size_t numCellsW_;
    size_t numCellsH_;
    size_t numKeypoints_;
    size_t numKeypoints2d_;
    size_t numKeypoints3d_;

    // Pose (T cam -> world), (T world -> cam)
    Sophus::SE3d Twc_;
    Sophus::SE3d Tcw_;

    std::shared_ptr<CameraCalibration> cameraCalibration_;

    Eigen::Matrix3d Frl_;
    cv::Mat Fcv_;

    std::map<int, int> covisibleKeyframeIds_;

    std::unordered_set<int> localMapPointIds_;
};
