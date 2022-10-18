#include "feature_tracker.hpp"
#include <unordered_map>
#include <opencv2/video/tracking.hpp>

void FeatureTracker::fbKltTracking(const std::vector<cv::Mat> &prevPyramid, const std::vector<cv::Mat> &currPyramid,
                                   int winSize, int numPyramidLevels, float errorValue, float maxFbkltDistance, std::vector<cv::Point2f> &points,
                                   std::vector<cv::Point2f> &priorKeypoints, std::vector<bool> &keypointStatus) const
{
    assert(prevPyramid.size() == currPyramid.size());

    if (points.empty())
    {
        return;
    }

    cv::Size kltWinSize(winSize, winSize);

    if ((int) prevPyramid.size() < 2 * (numPyramidLevels + 1))
    {
        numPyramidLevels = prevPyramid.size() / 2 - 1;
    }

    // Objects for OpenCV KLT
    size_t numKeypoints = points.size();
    keypointStatus.reserve(numKeypoints);

    std::vector<uchar> status;
    std::vector<float> errors;
    std::vector<int> keypointIndex;
    status.reserve(numKeypoints);
    errors.reserve(numKeypoints);
    keypointIndex.reserve(numKeypoints);

    // Tracking Forward
    cv::calcOpticalFlowPyrLK(prevPyramid, currPyramid, points, priorKeypoints, status, errors,
                             kltWinSize, numPyramidLevels, kltConvCriteria_,
                             (cv::OPTFLOW_USE_INITIAL_FLOW + cv::OPTFLOW_LK_GET_MIN_EIGENVALS)
    );

    std::vector<cv::Point2f> newKeypoints;
    std::vector<cv::Point2f> backKeypoints;
    newKeypoints.reserve(numKeypoints);
    backKeypoints.reserve(numKeypoints);

    size_t numGood = 0;

    // Init outliers vector & update tracked kps
    for (size_t i = 0; i < numKeypoints; i++)
    {
        if (!status.at(i))
        {
            keypointStatus.push_back(false);
            continue;
        }

        if (errors.at(i) > errorValue)
        {
            keypointStatus.push_back(false);
            continue;
        }

        if (!inBorder(priorKeypoints.at(i), currPyramid.at(0)))
        {
            keypointStatus.push_back(false);
            continue;
        }

        newKeypoints.push_back(priorKeypoints.at(i));
        backKeypoints.push_back(points.at(i));
        keypointStatus.push_back(true);
        keypointIndex.push_back(i);
        numGood++;
    }

    if (newKeypoints.empty())
    {
        return;
    }

    status.clear();
    errors.clear();

    // Tracking Backward
    cv::calcOpticalFlowPyrLK(currPyramid, prevPyramid, newKeypoints, backKeypoints, status, errors,
                             kltWinSize, 0, kltConvCriteria_,
                             (cv::OPTFLOW_USE_INITIAL_FLOW + cv::OPTFLOW_LK_GET_MIN_EIGENVALS)
    );

    numGood = 0;

    int n = newKeypoints.size();

    for (int i = 0; i < n; i++)
    {
        int idx = keypointIndex.at(i);

        if (!status.at(i))
        {
            keypointStatus.at(idx) = false;
            continue;
        }

        if (cv::norm(points.at(idx) - backKeypoints.at(i)) > maxFbkltDistance)
        {
            keypointStatus.at(idx) = false;
            continue;
        }

        numGood++;
    }
}

bool FeatureTracker::inBorder(const cv::Point2f &point, const cv::Mat &image) const
{
    const float BORDER_SIZE = 1.0;

    // True if point is within image borders
    return BORDER_SIZE <= point.x && point.x < image.cols - BORDER_SIZE && BORDER_SIZE <= point.y && point.y < image.rows - BORDER_SIZE;
}
