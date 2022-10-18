#include "ceres_parametrization.hpp"


namespace DirectSE3
{
    bool ReprojectionErrorKSE3XYZ::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
        // [fx, fy, cx, cy]
        Eigen::Map<const Eigen::Vector4d> lcalib(parameters[0]);

        // [tx, ty, tz, qw, qx, qy, qz]
        Eigen::Map<const Eigen::Vector3d> twc(parameters[1]);
        Eigen::Map<const Eigen::Quaterniond> qwc(parameters[1] + 3);

        Sophus::SE3d Twc(qwc, twc);
        Sophus::SE3d Tcw = Twc.inverse();

        // [x,y,z]
        Eigen::Map<const Eigen::Vector3d> wpt(parameters[2]);

        // Compute left/right reproj err
        Eigen::Vector2d pred;

        Eigen::Vector3d lcampt = Tcw * wpt;

        const double linvz = 1. / lcampt.z();

        pred << lcalib(0) * lcampt.x() * linvz + lcalib(2),
                lcalib(1) * lcampt.y() * linvz + lcalib(3);

        Eigen::Map<Eigen::Vector2d> werr(residuals);
        werr = sqrt_info_ * (pred - unpx_);

        // Update chi2err and depthpos info for post optim checking
        chi2err_ = 0.;
        for (int i = 0; i < 2; i++)
        {
            chi2err_ += std::pow(residuals[i], 2);
        }

        isDepthPositive_ = true;
        if (lcampt.z() <= 0)
        {
            isDepthPositive_ = false;
        }

        if (jacobians != NULL)
        {
            const double linvz2 = linvz * linvz;

            Eigen::Matrix<double, 2, 3, Eigen::RowMajor> J_lcam;
            J_lcam << linvz * lcalib(0), 0., -lcampt.x() * linvz2 * lcalib(0),
                    0., linvz * lcalib(1), -lcampt.y() * linvz2 * lcalib(1);

            Eigen::Matrix<double, 2, 3> J_lRcw;

            if (jacobians[1] != NULL || jacobians[2] != NULL)
            {
                J_lRcw.noalias() = J_lcam * Tcw.rotationMatrix();
            }

            if (jacobians[0] != NULL)
            {
                Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor> > J_lcalib(jacobians[0]);
                J_lcalib.setZero();
                J_lcalib(0, 0) = lcampt.x() * linvz;
                J_lcalib(0, 2) = 1.;
                J_lcalib(1, 1) = lcampt.y() * linvz;
                J_lcalib(1, 3) = 1.;

                J_lcalib = sqrt_info_ * J_lcalib.eval();
            }
            if (jacobians[1] != NULL)
            {
                Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor> > J_se3pose(jacobians[1]);
                J_se3pose.setZero();

                J_se3pose.block<2, 3>(0, 0) = -1. * J_lRcw;
                J_se3pose.block<2, 3>(0, 3).noalias() = J_lRcw * Sophus::SO3d::hat(wpt);

                J_se3pose = sqrt_info_ * J_se3pose.eval();
            }
            if (jacobians[2] != NULL)
            {
                Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor> > J_wpt(jacobians[2]);
                J_wpt.setZero();
                J_wpt.block<2, 3>(0, 0) = J_lRcw;

                J_wpt = sqrt_info_ * J_wpt.eval();
            }
        }

        return true;
    }

    bool ReprojectionErrorSE3::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
        // [tx, ty, tz, qw, qx, qy, qz]
        Eigen::Map<const Eigen::Vector3d> twc(parameters[0]);
        Eigen::Map<const Eigen::Quaterniond> qwc(parameters[0] + 3);

        Sophus::SE3d Twc(qwc, twc);
        Sophus::SE3d Tcw = Twc.inverse();

        // Compute left/right reproj err
        Eigen::Vector2d pred;

        Eigen::Vector3d lcampt = Tcw * wpt_;

        const double linvz = 1. / lcampt.z();

        pred << fx_ * lcampt.x() * linvz + cx_,
                fy_ * lcampt.y() * linvz + cy_;

        Eigen::Map<Eigen::Vector2d> werr(residuals);
        werr = sqrt_info_ * (pred - unpx_);

        // Update chi2err and depthpos info for post optim checking
        chi2err_ = 0.;
        for (int i = 0; i < 2; i++)
        {
            chi2err_ += std::pow(residuals[i], 2);
        }

        isDepthPositive_ = true;
        if (lcampt.z() <= 0)
        {
            isDepthPositive_ = false;
        }

        if (jacobians != NULL)
        {
            const double linvz2 = linvz * linvz;

            Eigen::Matrix<double, 2, 3, Eigen::RowMajor> J_lcam;
            J_lcam << linvz * fx_, 0., -lcampt.x() * linvz2 * fx_,
                    0., linvz * fy_, -lcampt.y() * linvz2 * fy_;

            Eigen::Matrix<double, 2, 3> J_lRcw;
            J_lRcw.noalias() = J_lcam * Tcw.rotationMatrix();

            if (jacobians[0] != NULL)
            {
                Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor> > J_se3pose(jacobians[0]);
                J_se3pose.setZero();

                J_se3pose.block<2, 3>(0, 0).noalias() = -1. * J_lRcw;
                J_se3pose.block<2, 3>(0, 3).noalias() = J_lRcw * Sophus::SO3d::hat(wpt_);

                J_se3pose = sqrt_info_ * J_se3pose.eval();
            }
        }

        return true;
    }

    bool ReprojectionErrorKSE3AnchInvDepth::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
        // [fx, fy, cx, cy]
        Eigen::Map<const Eigen::Vector4d> lcalib(parameters[0]);

        // [tx, ty, tz, qw, qx, qy, qz]
        Eigen::Map<const Eigen::Vector3d> twanch(parameters[1]);
        Eigen::Map<const Eigen::Quaterniond> qwanch(parameters[1] + 3);

        Eigen::Map<const Eigen::Vector3d> twc(parameters[2]);
        Eigen::Map<const Eigen::Quaterniond> qwc(parameters[2] + 3);

        Sophus::SE3d Twanch(qwanch, twanch);
        Sophus::SE3d Twc(qwc, twc);
        Sophus::SE3d Tcw = Twc.inverse();

        // [1/z_anch]
        const double zanch = 1. / parameters[3][0];

        Eigen::Matrix3d invK, K;
        K << lcalib(0), 0., lcalib(2),
                0., lcalib(1), lcalib(3),
                0., 0., 1.;
        invK = K.inverse();

        Eigen::Vector3d anchpt = zanch * invK * anchpx_;
        Eigen::Vector3d wpt = Twanch * anchpt;

        // Compute left/right reproj err
        Eigen::Vector2d pred;

        Eigen::Vector3d lcampt = Tcw * wpt;

        const double linvz = 1. / lcampt.z();

        pred << lcalib(0) * lcampt.x() * linvz + lcalib(2),
                lcalib(1) * lcampt.y() * linvz + lcalib(3);

        Eigen::Map<Eigen::Vector2d> werr(residuals);
        werr = sqrt_info_ * (pred - unpx_);

        // Update chi2err and depthpos info for post optim checking
        chi2err_ = 0.;
        for (int i = 0; i < 2; i++)
        {
            chi2err_ += std::pow(residuals[i], 2);
        }

        isDepthPositive_ = true;
        if (lcampt.z() <= 0)
        {
            isDepthPositive_ = false;
        }

        if (jacobians != NULL)
        {
            const double linvz2 = linvz * linvz;

            Eigen::Matrix<double, 2, 3, Eigen::RowMajor> J_lcam;
            J_lcam << linvz * lcalib(0), 0., -lcampt.x() * linvz2 * lcalib(0),
                    0., linvz * lcalib(1), -lcampt.y() * linvz2 * lcalib(1);

            Eigen::Matrix<double, 2, 3> J_lRcw;

            if (jacobians[1] != NULL || jacobians[2] != NULL || jacobians[3] != NULL)
            {
                Eigen::Matrix3d Rcw = Tcw.rotationMatrix();
                J_lRcw.noalias() = J_lcam * Rcw;
            }

            if (jacobians[0] != NULL)
            {
                Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor> > J_lcalib(jacobians[0]);
                J_lcalib.setZero();

                // TODO
            }
            if (jacobians[1] != NULL)
            {
                Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor> > J_se3anch(jacobians[1]);
                J_se3anch.setZero();

                Eigen::Matrix3d skew_wpt = Sophus::SO3d::hat(wpt);

                J_se3anch.block<2, 3>(0, 0) = J_lRcw;
                J_se3anch.block<2, 3>(0, 3).noalias() = -1. * J_lRcw * skew_wpt;

                J_se3anch = sqrt_info_ * J_se3anch.eval();
            }
            if (jacobians[2] != NULL)
            {
                Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor> > J_se3pose(jacobians[2]);
                J_se3pose.setZero();

                Eigen::Matrix3d skew_wpt = Sophus::SO3d::hat(wpt);

                J_se3pose.block<2, 3>(0, 0) = -1. * J_lRcw;
                J_se3pose.block<2, 3>(0, 3).noalias() = J_lRcw * skew_wpt;

                J_se3pose = sqrt_info_ * J_se3pose.eval();
            }
            if (jacobians[3] != NULL)
            {
                Eigen::Map<Eigen::Vector2d> J_invpt(jacobians[3]);
                Eigen::Vector3d J_lambda = -1. * zanch * Twanch.rotationMatrix() * anchpt;
                J_invpt.noalias() = J_lRcw * J_lambda;

                J_invpt = sqrt_info_ * J_invpt.eval();
            }
        }

        return true;
    }
}