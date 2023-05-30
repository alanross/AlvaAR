#pragma once

#include <unordered_map>

#include "state.hpp"
#include "frame.hpp"
#include "map_point.hpp"
#include "feature_extractor.hpp"
#include "feature_tracker.hpp"


class MapManager
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MapManager()
    {}

    MapManager(std::shared_ptr<State> state, std::shared_ptr<Frame> frame, std::shared_ptr<FeatureExtractor> featureExtractor);

    // Prepare Frame to become a keyframe (Update observations between map points / keyframes)
    void prepareFrame();

    // This function copies cur. Frame to add it to the keyframe map
    void addKeyframe();

    // adds a new map point to the map
    void addMapPoint(const cv::Scalar &color = cv::Scalar(200));

    // adds a new map point to the map with desc
    void addMapPoint(const cv::Mat &desc, const cv::Scalar &color = cv::Scalar(200));

    // Returns a shared_ptr of the req. keyframe
    std::shared_ptr<Frame> getKeyframe(const int KeyframeId) const;

    // Returns a shared_ptr of the required map point
    std::shared_ptr<MapPoint> getMapPoint(const int mapPointId) const;

    std::vector<Eigen::Vector3d> getCurrentFrameMapPoints() const;

    // Update a map point world position
    void updateMapPoint(const int mapPointId, const Eigen::Vector3d &wpt, const double keyframeAnchorInvDepth = -1.);

    bool setMapPointObs(const int mapPointId);

    void updateFrameCovisibility(Frame &frame);

    void mergeMapPoints(const int prvMapPointId, const int newMapPointId);

    // Remove a keyframe from the map
    void removeKeyframe(const int keyframeId);

    // Remove a map point from the map
    void removeMapPoint(const int mapPointId);

    // Remove a keyframe obs from a map point
    void removeMapPointObs(const int mapPointId, const int keyframeId);

    // Remove a map point obs from cur Frame
    void removeObsFromCurrFrameById(const int mapPointId);

    // turns current frame into a keyframe. Keypoints extraction is performed and the related map points and the new keyframe are added to the map.
    void createKeyframe(const cv::Mat &image, const cv::Mat &imageRaw);

    void addKeypointsToFrame(const cv::Mat &image, const std::vector<cv::Point2f> &points, const std::vector<cv::Mat> &descriptors, Frame &frame);

    // Extract new kps into provided image and update cur. Frame
    void extractKeypoints(const cv::Mat &image, const cv::Mat &imageRaw);

    // Describe cur frame kps in cur image
    void describeKeypoints(const cv::Mat &image, const std::vector<Keypoint> &keypoints, const std::vector<cv::Point2f> &points);

    void reset();

    int numMapPointIds_;
    int numKeyframeIds_;
    int numMapPoints_;
    int numKeyframes_;

    std::shared_ptr<State> state_;
    std::shared_ptr<Frame> currFrame_;
    std::shared_ptr<FeatureExtractor> featureExtractor_;

    std::unordered_map<int, std::shared_ptr<Frame>> mapKeyframes_;
    std::unordered_map<int, std::shared_ptr<MapPoint>> mapMapPoints_;

    std::vector<Point3D> pointCloud_;
};