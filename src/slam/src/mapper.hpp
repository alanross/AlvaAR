#pragma once

#include <queue>
#include <vector>
#include <unordered_set>
#include "map_manager.hpp"
#include "multi_view_geometry.hpp"
#include "optimizer.hpp"
#include "estimator.hpp"

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

    void displayInfo()
    {
        std::cout << "\n\n Keyframe id #" << keyframeId_;
        std::cout << " - image size : " << image_.size;
        std::cout << " - pyramid size : " << imagePyramid_.size() << "\n\n";
    }

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

    bool matchingToLocalMap(Frame &frame);

    std::map<int, int> matchToMap(const Frame &frame, const float maxProjectionError, const float distRatio, std::unordered_set<int> &localMapPointIds);

    void mergeMatches(const Frame &frame, const std::map<int, int> &mapOfKeypointIdsAndMapPointIds);

    void triangulateTemporal(Frame &frame);

    bool getNewKeyframe(Keyframe &keyframe);

    void addNewKeyframe(const Keyframe &keyframe);

    void reset();

    std::shared_ptr<State> state_;
    std::shared_ptr<Frame> currFrame_;
    std::shared_ptr<MapManager> mapManager_;
    std::shared_ptr<Estimator> estimator_;
    std::queue<Keyframe> keyframeQueue_;

    bool newKeyframeAvailable_ = false;
};