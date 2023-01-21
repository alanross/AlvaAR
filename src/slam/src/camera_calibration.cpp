#include "camera_calibration.hpp"

CameraCalibration::CameraCalibration(double fx, double fy, double cx, double cy, double k1, double k2, double p1, double p2, double imgWidth, double imgHeight, double imgBorder)
        : fx_(fx), fy_(fy), cx_(cx), cy_(cy), k1_(k1), k2_(k2), p1_(p1), p2_(p2), imgWidth_(imgWidth), imgHeight_(imgHeight), imgBorder_(imgBorder)
{
    // intrinsics
    K_ << fx_, 0., cx_, 0., fy_, cy_, 0., 0., 1.;

    // dist coefficient
    D_ << k1_, k2_, p1_, p2_;

    cv::eigen2cv(K_, Kcv_);
    cv::eigen2cv(D_, Dcv_);

    inverseK_ = K_.inverse();

    cv::eigen2cv(Tc0ci_.rotationMatrix(), Rcv_c0ci_);
    cv::eigen2cv(Tc0ci_.translation(), tcv_c0ci_);

    roi_rect_ = cv::Rect(cv::Point2i(imgBorder_, imgBorder_), cv::Point2i(imgWidth_ - imgBorder_, imgHeight_ - imgBorder_));
    roi_mask_ = cv::Mat(imgHeight, imgWidth, CV_8UC1, cv::Scalar(0));
    roi_mask_(roi_rect_) = 255;
}

cv::Point2f CameraCalibration::projectCamToImage(const Eigen::Vector3d &point) const
{
    double inverseZ = 1. / point.z();
    double x = point.x() * inverseZ;
    double y = point.y() * inverseZ;

    return cv::Point2f(fx_ * x + cx_, fy_ * y + cy_);
}

cv::Point2f CameraCalibration::projectCamToImageDist(const Eigen::Vector3d &point) const
{
    double inverseZ = 1. / point.z();
    double x = point.x() * inverseZ;
    double y = point.y() * inverseZ;

    if (Dcv_.empty())
    {
        return cv::Point2f(fx_ * x + cx_, fy_ * y + cy_);
    }

    cv::Point3f cvPoint(x, y, 1.0);
    std::vector<cv::Point3f> vPoint;
    vPoint.push_back(cvPoint);
    std::vector<cv::Point2f> pointDistances;

    cv::Mat R = cv::Mat::zeros(3, 1, CV_32F);

    cv::projectPoints(vPoint, R, R, Kcv_, Dcv_, pointDistances);

    return pointDistances[0];
}

cv::Point2f CameraCalibration::undistortImagePoint(const cv::Point2f &point) const
{
    if (Dcv_.empty())
    {
        return point;
    }

    std::vector<cv::Point2f> points;
    std::vector<cv::Point2f> undistortedPoints;

    points.push_back(point);

    cv::undistortPoints(points, undistortedPoints, Kcv_, Dcv_, Kcv_);

    return undistortedPoints[0];
}

Eigen::Matrix3d CameraCalibration::getRotation() const
{
    return Tc0ci_.rotationMatrix();
}

Eigen::Vector3d CameraCalibration::getTranslation() const
{
    return Tc0ci_.translation();
}