#pragma once

#include <unordered_map>
#include <set>
#include <Eigen/Core>
#include <opencv2/core.hpp>

struct Point3D
{
    float x;
    float y;
    float z;

    uint8_t r;
    uint8_t g;
    uint8_t b;

    Point3D() : x(0.0), y(0.0), z(0.0), r(0), g(0), b(0)
    {}

    Point3D(float _x, float _y, float _z, std::uint8_t _r, std::uint8_t _g, std::uint8_t _b) : x(_x), y(_y), z(_z), r(_r), g(_g), b(_b)
    {}
};

class MapPoint
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MapPoint()
    {}

    MapPoint(const int mapPointId, const int keyframeId, const bool observed = true);

    MapPoint(const int mapPointId, const int keyframeId, const cv::Mat &descriptor, const bool observed = true);

    MapPoint(const int mapPointId, const int keyframeId, const cv::Scalar &color, const bool observed = true);

    MapPoint(const int mapPointId, const int keyframeId, const cv::Mat &descriptor, const cv::Scalar &color, const bool observed = true);

    void setPoint(const Eigen::Vector3d &point3d, const double keyframeAnchorInvDepth = -1.0);

    Eigen::Vector3d getPoint() const;

    std::set<int> getObservedKeyframeIds() const;

    void addObservedKeyframeId(const int keyframeId);

    void removeObservedKeyframeId(const int keyframeId);

    void addDesc(const int keyframeId, const cv::Mat &descriptor);

    bool isBad();

    float computeMinDescDist(const MapPoint &mapPoint);

    // for using map point in ordered containers
    bool operator<(const MapPoint &mp) const
    {
        return mapPointId_ < mp.mapPointId_;
    }

    int mapPointId_;

    // true if seen in current frame
    bool isObserved_;

    // true if map point has been init
    bool is3d_;

    // observed keyframe ids
    std::set<int> observedKeyframeIds_;

    // 3d position
    Eigen::Vector3d point3d_;

    // anchored position
    int keyframeId_;
    double invDepth_;

    cv::Mat desc_;
    std::unordered_map<int, cv::Mat> mapKeyframeDescriptors_;
    std::unordered_map<int, float> mapDescriptorsDist_;

    cv::Scalar color_ = cv::Scalar(200);
};