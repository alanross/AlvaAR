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

    std::vector<cv::Mat> describeBRIEF(const cv::Mat &image, const std::vector<cv::Point2f> &points) const;

    std::vector<cv::Point2f> detectSingleScale(const cv::Mat &image, const int cellSize, const std::vector<cv::Point2f> &currKeypoints, const cv::Rect &roi);

    double maxQuality_;
};