#include "utils.hpp"

void Utils::toPoseArray(Sophus::SE3d Twc, float *pose)
{
    Eigen::Matrix3d R = Twc.rotationMatrix();
    Eigen::Vector3d t = Twc.translation();

    pose[0] = R(0, 0);
    pose[1] = R(0, 1);
    pose[2] = R(0, 2);
    pose[3] = 0.0;

    pose[4] = R(1, 0);
    pose[5] = R(1, 1);
    pose[6] = R(1, 2);
    pose[7] = 0.0;

    pose[8] = R(2, 0);
    pose[9] = R(2, 1);
    pose[10] = R(2, 2);
    pose[11] = 0.0;

    pose[12] = t.x();
    pose[13] = t.y();
    pose[14] = t.z();
    pose[15] = 1.0;
}

void Utils::toPoseArray(cv::Mat mat, float *pose)
{
    pose[0] = mat.at<float>(0, 0);
    pose[1] = mat.at<float>(1, 0);
    pose[2] = mat.at<float>(2, 0);
    pose[3] = 0.0;

    pose[4] = mat.at<float>(0, 1);
    pose[5] = mat.at<float>(1, 1);
    pose[6] = mat.at<float>(2, 1);
    pose[7] = 0.0;

    pose[8] = mat.at<float>(0, 2);
    pose[9] = mat.at<float>(1, 2);
    pose[10] = mat.at<float>(2, 2);
    pose[11] = 0.0;

    pose[12] = mat.at<float>(0, 3);
    pose[13] = mat.at<float>(1, 3);
    pose[14] = mat.at<float>(2, 3);
    pose[15] = 1.0;
}

void Utils::toPoseMat(Sophus::SE3d Twc, cv::Mat &pose)
{
    Eigen::Matrix3d R = Twc.rotationMatrix();
    Eigen::Vector3d t = Twc.translation();

    pose.at<float>(0, 0) = R(0, 0);
    pose.at<float>(0, 1) = R(1, 0);
    pose.at<float>(0, 2) = R(2, 0);
    pose.at<float>(0, 3) = t.x();

    pose.at<float>(1, 0) = R(0, 1);
    pose.at<float>(1, 1) = R(1, 1);
    pose.at<float>(1, 2) = R(2, 1);
    pose.at<float>(1, 3) = t.y();

    pose.at<float>(2, 0) = R(0, 2);
    pose.at<float>(2, 1) = R(1, 2);
    pose.at<float>(2, 2) = R(2, 2);
    pose.at<float>(2, 3) = t.z();

    pose.at<float>(3, 0) = 0.0;
    pose.at<float>(3, 1) = 0.0;
    pose.at<float>(3, 2) = 0.0;
    pose.at<float>(3, 3) = 1.0;
}