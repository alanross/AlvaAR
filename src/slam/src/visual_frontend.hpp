#pragma once

#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "state.hpp"
#include "map_manager.hpp"
#include "mapper.hpp"
#include "feature_tracker.hpp"

class MotionModel
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void applyMotionModel(Sophus::SE3d &Twc, double time)
    {
        if (prevTime_ > 0)
        {
            // Provided Twc and prevTwc should be equal here as prevTwc is updated right after pose computation
            if (!(Twc * prevTwc_.inverse()).log().isZero(1.e-5))
            {
                // Might happen in case of LC! So update prevPose to stay consistent
                prevTwc_ = Twc;
            }

            double dt = (time - prevTime_);
            Twc = Twc * Sophus::SE3d::exp(logRelT_ * dt);
        }
    }

    void updateMotionModel(const Sophus::SE3d &Twc, double time)
    {
        if (prevTime_ < 0.)
        {
            prevTime_ = time;
            prevTwc_ = Twc;
        }
        else
        {
            double dt = time - prevTime_;

            prevTime_ = time;

            if (dt < 0.)
            {
                std::cerr << "\nGot image older than previous image! LEAVING!\n";
                exit(-1);
            }

            Sophus::SE3d Tprevcur = prevTwc_.inverse() * Twc;
            logRelT_ = Tprevcur.log() / dt;
            prevTwc_ = Twc;
        }
    }

    void reset()
    {
        prevTime_ = -1.;
        logRelT_ = Eigen::Matrix<double, 6, 1>::Zero();
    }

    double prevTime_ = -1.;

    Sophus::SE3d prevTwc_;
    Eigen::Matrix<double, 6, 1> logRelT_ = Eigen::Matrix<double, 6, 1>::Zero();
};

class VisualFrontend
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VisualFrontend()
    {}

    VisualFrontend(std::shared_ptr<State> state, std::shared_ptr<Frame> frame, std::shared_ptr<MapManager> mapManager, std::shared_ptr<Mapper> mapper, std::shared_ptr<FeatureTracker> featureTracker);

    void track(cv::Mat &image, double timestamp);

    void reset();

private:
    bool process(cv::Mat &image, double timestamp);

    void preprocessImage(cv::Mat &image);

    void kltTrackingFromMotionPrior();

    bool computePose();

    // compute parallax (in px.) between current frame and the provided keyframe
    float computeParallax(const int keyframeId, bool doUnRotate, bool doMedian);

    bool checkReadyForInit();

    bool checkNewKeyframeRequired();

    void resetFrame();

    std::shared_ptr<State> state_;
    std::shared_ptr<Frame> currFrame_;
    std::shared_ptr<MapManager> mapManager_;
    std::shared_ptr<Mapper> mapper_;
    std::shared_ptr<FeatureTracker> featureTracker_;

    cv::Ptr<cv::CLAHE> clahe_;
    cv::Mat currImage_;
    cv::Mat prevImage_;
    std::vector<cv::Mat> currPyramid_;
    std::vector<cv::Mat> prevPyramid_;
    std::vector<cv::Mat> keyframePyramid_;

    MotionModel motionModel_;

    bool p3pReq_ = false;
    int poseFailedCounter_ = 0;
};
