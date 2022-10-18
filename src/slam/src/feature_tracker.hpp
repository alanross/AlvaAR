#pragma once

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

class FeatureTracker
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FeatureTracker(int maxIterations, float maxPxPrecision) : kltConvCriteria_(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, maxIterations, maxPxPrecision)
    {}

    // Forward-Backward KLT Tracking
    void fbKltTracking(
            const std::vector<cv::Mat> &prevPyramid,
            const std::vector<cv::Mat> &currPyramid,
            int winSize,
            int numPyramidLevels,
            float errorValue,
            float maxFbkltDistance,
            std::vector<cv::Point2f> &points,
            std::vector<cv::Point2f> &priorKeypoints,
            std::vector<bool> &keypointStatus
    ) const;

    bool inBorder(const cv::Point2f &point, const cv::Mat &image) const;

    // KLT optimization parameter
    cv::TermCriteria kltConvCriteria_;
};