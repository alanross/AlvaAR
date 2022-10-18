#pragma once

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <opencv2/core.hpp>

class MultiViewGeometry
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Triangulation

    static Eigen::Vector3d triangulate(const Sophus::SE3d &Tlr, const Eigen::Vector3d &bvl, const Eigen::Vector3d &bvr);

    // P3P - PnP methods

    static bool p3pRansac(
            const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &bvs,
            const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &vwpts,
            const int maxIterations,
            const float errorThreshold,
            const bool optimize,
            const bool doRandom,
            const float fx,
            const float fy,
            Sophus::SE3d &Twc,
            std::vector<int> &outliersIndices,
            bool useP3PLMeds = false
    );

    static bool opengvP3PRansac(
            const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &observations,
            const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &wPoints,
            const int maxIterations,
            const float errorThreshold,
            const bool optimize,
            const bool doRandom,
            const float fx,
            const float fy,
            Sophus::SE3d &Twc,
            std::vector<int> &outliersIndices
    );

    static bool opengvP3PLMeds(
            const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &observations,
            const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &wPoints,
            const int maxIterations,
            const float errorThreshold,
            const bool optimize,
            const bool doRandom,
            const float fx,
            const float fy,
            Sophus::SE3d &Twc,
            std::vector<int> &outliersIndices
    );

    static bool ceresPnP(
            const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > &unKeypoints,
            const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &wPoints,
            Sophus::SE3d &Twc,
            const int maxIterations,
            const float chi2th,
            const bool useRobust,
            const bool applyL2AfterRobust,
            const float fx,
            const float fy,
            const float cx,
            const float cy,
            std::vector<int> &outliersIndices
    );

    // 2D-2D Epipolar Geometry

    static bool compute5ptEssentialMatrix(
            const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &observations1,
            const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &observations2,
            const int maxIterations,
            const float errorThreshold,
            const bool optimize,
            const bool doRandom,
            const float fx,
            const float fy,
            Eigen::Matrix3d &Rwc,
            Eigen::Vector3d &twc,
            std::vector<int> &outliersIndices
    );
};
