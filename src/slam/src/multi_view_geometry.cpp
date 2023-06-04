#include "multi_view_geometry.hpp"
#include "ceres_parametrization.hpp"
#include <opengv/types.hpp>
#include <opengv/triangulation/methods.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac/Lmeds.hpp>
#include <opengv/absolute_pose/CentralAbsoluteAdapter.hpp>
#include <opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>

Eigen::Vector3d MultiViewGeometry::triangulate(const Sophus::SE3d &Tlr, const Eigen::Vector3d &bvl, const Eigen::Vector3d &bvr)
{
    opengv::bearingVectors_t bv1(1, bvl);
    opengv::bearingVectors_t bv2(1, bvr);
    opengv::rotation_t R12 = Tlr.rotationMatrix();
    opengv::translation_t t12 = Tlr.translation();
    opengv::relative_pose::CentralRelativeAdapter adapter(bv1, bv2, t12, R12);
    opengv::point_t pt = opengv::triangulation::triangulate2(adapter, 0);

    return pt;
}

bool MultiViewGeometry::p3pRansac(
        const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &observations,
        const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &wPoints,
        const int maxIterations,
        const float errorThreshold,
        const bool optimize,
        const bool doRandom,
        const float fx,
        const float fy,
        Sophus::SE3d &Twc,
        std::vector<int> &outliersIndices)
{
    assert(observations.size() == wPoints.size());

    size_t num3dPoints = observations.size();

    if (num3dPoints < 4)
    {
        return false;
    }

    outliersIndices.reserve(num3dPoints);

    opengv::bearingVectors_t gvbvs;
    opengv::points_t gvwpt;
    gvbvs.reserve(num3dPoints);
    gvwpt.reserve(num3dPoints);

    for (size_t i = 0; i < num3dPoints; i++)
    {
        gvbvs.push_back(observations.at(i));
        gvwpt.push_back(wPoints.at(i));
    }

    opengv::absolute_pose::CentralAbsoluteAdapter adapter(gvbvs, gvwpt);

    // Create an AbsolutePoseSac problem and Ransac
    opengv::sac::Lmeds<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem> ransac;
//    opengv::sac::Ransac<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem> ransac;

    std::shared_ptr<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem> absposeproblem_ptr(
            new opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem(
                    adapter,
                    opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem::KNEIP, // KNEIP, GAO or EPNP
                    doRandom)
    );

    float focal = fx + fy;
    focal /= 2.;

    ransac.sac_model_ = absposeproblem_ptr;
    ransac.threshold_ = (1.0 - cos(atan(errorThreshold / focal)));
    ransac.max_iterations_ = maxIterations;

    // Computing the pose from P3P
    ransac.computeModel(0);

    // If no solution found, return false
    if (ransac.inliers_.size() < 5)
    {
        return false;
    }

    // Might happen apparently...
    if (!Sophus::isOrthogonal(ransac.model_coefficients_.block<3, 3>(0, 0)))
    {
        return false;
    }

    // Optimize the computed pose with inliers only
    opengv::transformation_t T_opt;

    if (optimize)
    {
        ransac.sac_model_->optimizeModelCoefficients(ransac.inliers_, ransac.model_coefficients_, T_opt);

        Twc.translation() = T_opt.rightCols(1);
        Twc.setRotationMatrix(T_opt.leftCols(3));
    }
    else
    {
        Twc.translation() = ransac.model_coefficients_.rightCols(1);
        Twc.setRotationMatrix(ransac.model_coefficients_.leftCols(3));
    }

    size_t k = 0;
    for (size_t i = 0; i < num3dPoints; i++)
    {
        if (ransac.inliers_.at(k) == (int) i)
        {
            k++;
            if (k == ransac.inliers_.size())
            {
                k = 0;
            }
        }
        else
        {
            outliersIndices.push_back(i);
        }
    }

    return true;
}

bool MultiViewGeometry::ceresPnP(
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
        std::vector<int> &outliersIndices)
{
    assert(unKeypoints.size() == wPoints.size());

    ceres::Problem problem;

    double chi2ThresholdSqrt = std::sqrt(chi2th);

    ceres::LossFunctionWrapper *lossFunction;
    lossFunction = new ceres::LossFunctionWrapper(new ceres::HuberLoss(chi2ThresholdSqrt), ceres::TAKE_OWNERSHIP);

    if (!useRobust)
    {
        lossFunction->Reset(NULL, ceres::TAKE_OWNERSHIP);
    }

    size_t numKeyPoints = unKeypoints.size();

    ceres::LocalParameterization *local_parameterization = new SE3Parameterization();

    PoseParametersBlock posepar = PoseParametersBlock(0, Twc);

    problem.AddParameterBlock(posepar.values(), 7, local_parameterization);

    std::vector<DirectSE3::ReprojectionErrorSE3 *> verrors_;
    std::vector<ceres::ResidualBlockId> vrids_;

    const float scale = 1.0;

    for (size_t i = 0; i < numKeyPoints; i++)
    {
        DirectSE3::ReprojectionErrorSE3 *f = new DirectSE3::ReprojectionErrorSE3(unKeypoints[i].x(), unKeypoints[i].y(), fx, fy, cx, cy, wPoints.at(i), scale);

        ceres::ResidualBlockId rid = problem.AddResidualBlock(f, lossFunction, posepar.values());

        verrors_.push_back(f);
        vrids_.push_back(rid);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR; // DENSE_QR, DENSE_SCHUR or SPARSE_NORMAL_CHOLESKY
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.num_threads = 1;
    options.max_num_iterations = maxIterations;
    options.max_solver_time_in_seconds = 0.005;
    options.function_tolerance = 1.e-3;
    options.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    size_t nbbad = 0;

    for (size_t i = 0; i < numKeyPoints; i++)
    {
        auto err = verrors_.at(i);
        if (err->chi2err_ > chi2th || !err->isDepthPositive_)
        {
            if (applyL2AfterRobust)
            {
                auto rid = vrids_.at(i);
                problem.RemoveResidualBlock(rid);
            }
            outliersIndices.push_back(i);
            nbbad++;
        }
    }

    if (nbbad == numKeyPoints)
    {
        return false;
    }

    if (applyL2AfterRobust && !outliersIndices.empty())
    {
        lossFunction->Reset(NULL, ceres::TAKE_OWNERSHIP);
        ceres::Solve(options, &problem, &summary);
    }

    Twc = posepar.getPose();

    return summary.IsSolutionUsable();
}

bool MultiViewGeometry::compute5ptEssentialMatrix(
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
        std::vector<int> &outliersIndices)
{
    assert(observations1.size() == observations2.size());

    size_t numPoints = observations1.size();

    if (numPoints < 8)
    {
        return false;
    }

    outliersIndices.reserve(numPoints);

    opengv::bearingVectors_t bearingVectors1;
    opengv::bearingVectors_t bearingVectors2;
    bearingVectors1.reserve(numPoints);
    bearingVectors2.reserve(numPoints);

    for (size_t i = 0; i < numPoints; i++)
    {
        bearingVectors1.push_back(observations1.at(i));
        bearingVectors2.push_back(observations2.at(i));
    }

    //create a central relative adapter
    opengv::relative_pose::CentralRelativeAdapter adapter(bearingVectors1, bearingVectors2);

    opengv::sac::Ransac<opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem> ransac;

    std::shared_ptr<opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem> relposeproblem_ptr(
            new opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem(
                    adapter,
                    opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem::NISTER,
                    doRandom
            )
    );

    float focal = fx + fy;
    focal /= 2.;

    ransac.sac_model_ = relposeproblem_ptr;
    ransac.threshold_ = 2.0 * (1.0 - cos(atan(errorThreshold / focal)));
    ransac.max_iterations_ = maxIterations;

    ransac.computeModel(0);

    // If no solution found, return false
    if (ransac.inliers_.size() < 10)
    {
        return false;
    }

    twc = ransac.model_coefficients_.rightCols(1);
    Rwc = ransac.model_coefficients_.leftCols(3);

    // Optimize the computed pose with inliers only
    opengv::transformation_t T_opt;

    if (optimize)
    {
        ransac.sac_model_->optimizeModelCoefficients(ransac.inliers_, ransac.model_coefficients_, T_opt);

        Rwc = T_opt.leftCols(3);
        twc = T_opt.rightCols(1);
    }

    size_t k = 0;
    for (size_t i = 0; i < numPoints; i++)
    {
        if (ransac.inliers_.at(k) == (int) i)
        {
            k++;
            if (k == ransac.inliers_.size())
            {
                k = 0;
            }
        }
        else
        {
            outliersIndices.push_back(i);
        }
    }

    return true;
}
