#pragma once

#include <iostream>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>

#include <sophus/se3.hpp>

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

class CameraCalibration
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CameraCalibration()
    {}

    CameraCalibration(double fx, double fy, double cx, double cy, double k1, double k2, double p1, double p2, double imgWidth, double imgHeight, double imgBorder);

    cv::Point2f projectCamToImageDist(const Eigen::Vector3d &point) const;

    cv::Point2f projectCamToImage(const Eigen::Vector3d &point) const;

    cv::Point2f undistortImagePoint(const cv::Point2f &point) const;

    Eigen::Matrix3d getRotation() const;

    Eigen::Vector3d getTranslation() const;

    double fx_, fy_, cx_, cy_;
    double k1_, k2_, p1_, p2_;

    double imgWidth_;
    double imgHeight_;
    double imgBorder_;

    cv::Mat Kcv_;
    cv::Mat Dcv_;

    Eigen::Vector4d D_;
    Eigen::Matrix3d K_;
    Eigen::Matrix3d inverseK_;

    // Extrinsic Parameters
    Sophus::SE3d Tc0ci_;
    cv::Mat Rcv_c0ci_;
    cv::Mat tcv_c0ci_;

    // ROI Mask for detection
    cv::Rect roi_rect_;
    cv::Mat roi_mask_;
};
