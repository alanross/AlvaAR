#pragma once

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <ceres/ceres.h>
#include <cstddef>

class CalibParametersBlock
{

public:
    CalibParametersBlock()
    {}

    CalibParametersBlock(const int id, const double fx, const double fy, const double cx, const double cy)
    {
        id_ = id;
        values_[0] = fx;
        values_[1] = fy;
        values_[2] = cx;
        values_[3] = cy;
    }

    CalibParametersBlock(const CalibParametersBlock &block)
    {
        id_ = block.id_;

        for (size_t i = 0; i < numDimensions_; i++)
        {
            values_[i] = block.values_[i];
        }
    }

    CalibParametersBlock &operator=(const CalibParametersBlock &block)
    {
        id_ = block.id_;

        for (size_t i = 0; i < numDimensions_; i++)
        {
            values_[i] = block.values_[i];
        }
        return *this;
    }

    inline double *values()
    {
        return values_;
    }

    static const size_t numDimensions_ = 4;
    double values_[numDimensions_] = {0., 0., 0., 0.};
    int id_ = -1;
};

class PoseParametersBlock
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PoseParametersBlock()
    {}

    PoseParametersBlock(const int id, const Sophus::SE3d &T)
    {
        id_ = id;
        Eigen::Map<Eigen::Vector3d> t(values_);
        Eigen::Map<Eigen::Quaterniond> q(values_ + 3);
        t = T.translation();
        q = T.unit_quaternion();
    }

    PoseParametersBlock(const PoseParametersBlock &block)
    {
        id_ = block.id_;

        for (size_t i = 0; i < numDimensions_; i++)
        {
            values_[i] = block.values_[i];
        }
    }

    PoseParametersBlock &operator=(const PoseParametersBlock &block)
    {
        id_ = block.id_;

        for (size_t i = 0; i < numDimensions_; i++)
        {
            values_[i] = block.values_[i];
        }
        return *this;
    }

    Sophus::SE3d getPose()
    {
        Eigen::Map<Eigen::Vector3d> t(values_);
        Eigen::Map<Eigen::Quaterniond> q(values_ + 3);
        return Sophus::SE3d(q, t);
    }

    inline double *values()
    {
        return values_;
    }

    static const size_t numDimensions_ = 7;
    double values_[numDimensions_] = {0., 0., 0., 0., 0., 0., 0.};
    int id_ = 0.;
};

class PointXYZParametersBlock
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PointXYZParametersBlock()
    {}

    PointXYZParametersBlock(const int id, const Eigen::Vector3d &X)
    {
        id_ = id;
        Eigen::Map<Eigen::Vector3d>(values_, 3, 1) = X;
    }

    PointXYZParametersBlock(const PointXYZParametersBlock &block)
    {
        id_ = block.id_;

        for (size_t i = 0; i < numDimensions_; i++)
        {
            values_[i] = block.values_[i];
        }
    }

    PointXYZParametersBlock &operator=(const PointXYZParametersBlock &block)
    {
        id_ = block.id_;

        for (size_t i = 0; i < numDimensions_; i++)
        {
            values_[i] = block.values_[i];
        }
        return *this;
    }

    Eigen::Vector3d getPoint()
    {
        Eigen::Map<Eigen::Vector3d> X(values_);
        return X;
    }

    inline double *values()
    {
        return values_;
    }

    static const size_t numDimensions_ = 3;
    double values_[numDimensions_] = {0., 0., 0.};
    int id_ = -1;
};

class InvDepthParametersBlock
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    InvDepthParametersBlock()
    {}

    InvDepthParametersBlock(const int id, const int kfid, const double anch_depth) : id_(id), kfid_(kfid)
    {
        values_[0] = 1. / anch_depth;
    }

    InvDepthParametersBlock(const InvDepthParametersBlock &block)
    {
        id_ = block.id_;

        for (size_t i = 0; i < numDimensions_; i++)
        {
            values_[i] = block.values_[i];
        }
    }

    InvDepthParametersBlock &operator=(const InvDepthParametersBlock &block)
    {
        id_ = block.id_;

        for (size_t i = 0; i < numDimensions_; i++)
        {
            values_[i] = block.values_[i];
        }

        return *this;
    }

    double getInvDepth()
    {
        return values_[0];
    }

    inline double *values()
    {
        return values_;
    }

    static const size_t numDimensions_ = 1;
    double values_[numDimensions_] = {0.};
    int id_ = -1;
    int kfid_ = -1;
};

/*
    SE(3) Parametrization such as:
    1. T + dT = Exp(dT) * T
    2. T o X = T^(-1) * X (i.e. T: cam -> world)
*/
class SE3Parameterization : public ceres::LocalParameterization
{

public:
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const
    {
        Eigen::Map<const Eigen::Vector3d> t(x);
        Eigen::Map<const Eigen::Quaterniond> q(x + 3);

        Eigen::Map<const Eigen::Matrix<double, 6, 1>> vdelta(delta);

        Sophus::SE3d upT = Sophus::SE3d::exp(vdelta) * Sophus::SE3d(q, t);

        Eigen::Map<Eigen::Vector3d> upt(x_plus_delta);
        Eigen::Map<Eigen::Quaterniond> upq(x_plus_delta + 3);

        upt = upT.translation();
        upq = upT.unit_quaternion();

        return true;
    }

    virtual bool ComputeJacobian(const double *x, double *jacobian) const
    {
        Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor> > J(jacobian);
        J.topRows<6>().setIdentity();
        J.bottomRows<1>().setZero();
        return true;
    }

    virtual int GlobalSize() const
    { return 7; }

    virtual int LocalSize() const
    { return 6; }
};

// Cost functions with SE(3) pose parametrized as T cam -> world
namespace DirectSE3
{
    class ReprojectionErrorKSE3XYZ : public ceres::SizedCostFunction<2, 4, 7, 3>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ReprojectionErrorKSE3XYZ(const double u, const double v, const double sigma = 1.) : unpx_(u, v)
        {
            sqrt_cov_ = sigma * Eigen::Matrix2d::Identity();
            sqrt_info_ = sqrt_cov_.inverse();
        }

        virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

        // Mutable variable that will be updated in const Evaluate()
        mutable double chi2err_;
        mutable bool isDepthPositive_;
        Eigen::Matrix2d sqrt_cov_;
        Eigen::Matrix2d sqrt_info_;
    private:
        Eigen::Vector2d unpx_;
    };

    class ReprojectionErrorSE3 : public ceres::SizedCostFunction<2, 7>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ReprojectionErrorSE3(const double u, const double v, double fx, double fy, double cx, double cy, const Eigen::Vector3d &wpt, const double sigma = 1.) : unpx_(u, v),
                                                                                                                                                                wpt_(wpt), fx_(fx),
                                                                                                                                                                fy_(fy), cx_(cx),
                                                                                                                                                                cy_(cy)
        {
            sqrt_cov_ = sigma * Eigen::Matrix2d::Identity();
            sqrt_info_ = sqrt_cov_.inverse();
        }

        virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

        // Mutable variable that will be updated in const Evaluate()
        mutable double chi2err_;
        mutable bool isDepthPositive_;
        Eigen::Matrix2d sqrt_cov_, sqrt_info_;
    private:
        Eigen::Vector2d unpx_;
        Eigen::Vector3d wpt_;
        double fx_, fy_, cx_, cy_;
    };

    class ReprojectionErrorKSE3AnchInvDepth : public ceres::SizedCostFunction<2, 4, 7, 7, 1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ReprojectionErrorKSE3AnchInvDepth(const double u, const double v, const double uanch, const double vanch, const double sigma = 1.) : unpx_(u, v), anchpx_(uanch, vanch, 1.)
        {
            sqrt_cov_ = sigma * Eigen::Matrix2d::Identity();
            sqrt_info_ = sqrt_cov_.inverse();
        }

        virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

        // Mutable variable that will be updated in const Evaluate()
        mutable double chi2err_;
        mutable bool isDepthPositive_;
        Eigen::Matrix2d sqrt_cov_, sqrt_info_;
    private:
        Eigen::Vector2d unpx_;
        Eigen::Vector3d anchpx_;
    };
}