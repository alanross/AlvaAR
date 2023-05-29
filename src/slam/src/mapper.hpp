#pragma once

#include <queue>
#include <vector>
#include <unordered_set>
#include "map_manager.hpp"
#include "multi_view_geometry.hpp"
#include "optimizer.hpp"

struct Keyframe
{
    int keyframeId_;
    cv::Mat image_;
    cv::Mat imageRaw_;
    std::vector<cv::Mat> imagePyramid_;

    Keyframe() : keyframeId_(-1)
    {}

    Keyframe(int keyframeId, const cv::Mat &imageRaw) : keyframeId_(keyframeId), imageRaw_(imageRaw.clone())
    {}

    void releaseImages()
    {
        image_.release();
        imageRaw_.release();
        imagePyramid_.clear();
    }
};

// Handles Keyframe processing (i.e. triangulation, local map tracking, BA, LC)
class Mapper
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Mapper()
    {}

    Mapper(std::shared_ptr<State> state, std::shared_ptr<MapManager> mapManager, std::shared_ptr<Frame> frame);

    void processNewKeyframe(const Keyframe &keyframe);

private:
    std::map<int, int> matchToMap(const Frame &frame, const float maxProjectionError, const float distRatio, std::unordered_set<int> &localMapPointIds);

    bool matchingToLocalMap(Frame &frame);

    void optimize(const std::shared_ptr<Frame> &keyframe);

    void mergeMatches(const Frame &frame, const std::map<int, int> &mapOfKeypointIdsAndMapPointIds);

    void triangulateTemporal(Frame &frame);

    void timedOperationStart();

    bool timedOperationHasTimedOut();

    std::shared_ptr<State> state_;
    std::shared_ptr<Frame> currFrame_;
    std::shared_ptr<MapManager> mapManager_;
    std::unique_ptr<Optimizer> optimizer_;

    std::chrono::high_resolution_clock::time_point timedOperationStartTime_;
};