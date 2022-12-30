#pragma once

#include "frame.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

class FeatureExtractor
{

public:
    FeatureExtractor()
    {};

    FeatureExtractor(double maxQuality);

    std::vector<cv::Point2f> detectFeaturePoints(const cv::Mat &image, const int cellSize, const std::vector<cv::Point2f> &currKeypoints, const cv::Rect &roi);

    std::vector<cv::Mat> describeFeaturePoints(const cv::Mat &image, const std::vector<cv::Point2f> &points) const;

private:
    double maxQuality_;
};