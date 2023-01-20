#pragma once

#include <opencv2/core.hpp>
#include <Eigen/Core>

class Utils
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static void toPoseArray(Eigen::Matrix3d rwc, Eigen::Vector3d twc, float *pose);

    static void toPoseArray(cv::Mat m, float *pose);

    static void toPoseMat(Eigen::Matrix3d rwc, Eigen::Vector3d twc, cv::Mat &pose);

    static cv::Mat expSO3(const cv::Mat &v);
};
