#pragma once

#include <opencv2/core.hpp>
#include <Eigen/Core>
#include <sophus/se3.hpp>

class Utils
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static void toPoseArray(Sophus::SE3d Twc, float *pose);

    static void toPoseArray(cv::Mat mat, float *pose);

    static void toPoseMat(Sophus::SE3d Twc, cv::Mat &pose);
};
