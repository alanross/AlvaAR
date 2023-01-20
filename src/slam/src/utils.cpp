#include "utils.hpp"

void Utils::toPoseArray(Eigen::Matrix3d rwc, Eigen::Vector3d twc, float *pose)
{
    pose[0] = rwc(0, 0);
    pose[1] = rwc(0, 1);
    pose[2] = rwc(0, 2);
    pose[3] = 0.0;

    pose[4] = rwc(1, 0);
    pose[5] = rwc(1, 1);
    pose[6] = rwc(1, 2);
    pose[7] = 0.0;

    pose[8] = rwc(2, 0);
    pose[9] = rwc(2, 1);
    pose[10] = rwc(2, 2);
    pose[11] = 0.0;

    pose[12] = twc.x();
    pose[13] = twc.y();
    pose[14] = twc.z();
    pose[15] = 1.0;
}

void Utils::toPoseArray(cv::Mat m, float *pose)
{
    pose[0] = m.at<float>(0, 0);
    pose[1] = m.at<float>(1, 0);
    pose[2] = m.at<float>(2, 0);
    pose[3] = 0.0;

    pose[4] = m.at<float>(0, 1);
    pose[5] = m.at<float>(1, 1);
    pose[6] = m.at<float>(2, 1);
    pose[7] = 0.0;

    pose[8] = m.at<float>(0, 2);
    pose[9] = m.at<float>(1, 2);
    pose[10] = m.at<float>(2, 2);
    pose[11] = 0.0;

    pose[12] = m.at<float>(0, 3);
    pose[13] = m.at<float>(1, 3);
    pose[14] = m.at<float>(2, 3);
    pose[15] = 1.0;
}

void Utils::toPoseMat(Eigen::Matrix3d rwc, Eigen::Vector3d twc, cv::Mat &pose)
{
    pose.at<float>(0, 0) = rwc(0, 0);
    pose.at<float>(0, 1) = rwc(1, 0);
    pose.at<float>(0, 2) = rwc(2, 0);
    pose.at<float>(0, 3) = twc.x();

    pose.at<float>(1, 0) = rwc(0, 1);
    pose.at<float>(1, 1) = rwc(1, 1);
    pose.at<float>(1, 2) = rwc(2, 1);
    pose.at<float>(1, 3) = twc.y();

    pose.at<float>(2, 0) = rwc(0, 2);
    pose.at<float>(2, 1) = rwc(1, 2);
    pose.at<float>(2, 2) = rwc(2, 2);
    pose.at<float>(2, 3) = twc.z();

    pose.at<float>(3, 0) = 0.0;
    pose.at<float>(3, 1) = 0.0;
    pose.at<float>(3, 2) = 0.0;
    pose.at<float>(3, 3) = 1.0;
}

cv::Mat Utils::expSO3(const cv::Mat &v)
{
    const float x = v.at<float>(0);
    const float y = v.at<float>(1);
    const float z = v.at<float>(2);

    cv::Mat I = cv::Mat::eye(3, 3, CV_32F);

    const float d2 = x * x + y * y + z * z;
    const float d = sqrt(d2);

    cv::Mat W = (cv::Mat_<float>(3, 3) << 0, -z, y, z, 0, -x, -y, x, 0);

    if (d < 0.0001)
    {
        return (I + W + 0.5f * W * W);
    }
    else
    {
        return (I + W * sin(d) / d + W * W * (1.0f - cos(d)) / d2);
    }
}