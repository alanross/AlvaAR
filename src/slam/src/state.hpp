#pragma once

#include <iostream>
#include <string>
#include <chrono>
#include <Eigen/Core>
#include <opencv2/core.hpp>

/*
   Values                       FAST:       AVERAGE:        ACCURATE:
   ----------------------------------------------------------------------------
   frameMaxCellSize_:           50          45              35
   claheEnabled_:               false       false           true
   mapKeyframeFilteringRatio:   0.9         0.9             0.95
   p3pEnabled_ :                true        false           false
   ----------------------------------------------------------------------------
*/

class State
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    State(double imgWidth, double imgHeight, int frameMaxNumKeypoints);

    void reset();

    double imgWidth_;
    double imgHeight_;

    bool debug_ = false;

    bool slamReadyForInit_ = false;
    bool slamResetRequested_ = false;

    float minAvgRotationParallax_ = 40.0;

    int frameMaxNumKeypoints_;
    int frameMaxCellSize_ = 35;

    // Image pre-processing
    bool claheEnabled_ = false;
    float claheContrastLimit_ = 3;
    int claheTileSize_ = 50;

    // KLT parameters
    bool kltEnabled_ = true;
    bool kltUsePrior_ = true;
    int kltPyramidLevels_ = 3;
    int kltError_ = 30.0;
    int kltWinSizeWH_ = 9;
    cv::Size kltWinSize_ = cv::Size(kltWinSizeWH_, kltWinSizeWH_);
    float kltMaxFbDistance_ = 0.5;

    // Image features
    int trackerMaxIterations_ = 30;
    float trackerMaxPxPrecision_ = 0.01;
    double extractorMaxQuality_ = 0.001;

    // Map Filtering parameters
    float mapMaxDescriptorDistance_ = 0.2;
    float mapMaxProjectionPxDistance_ = 2.0;
    float mapMaxReprojectionError_ = 3.0;
    float mapKeyframeFilteringRatio_ = 0.95;

    bool multiViewRandomEnabled_ = true;
    float multiViewRansacError_ = 3.0;
    int multiViewRansacNumIterations_ = 100;

    bool p3pEnabled_ = true;

    // Bundle Adjustment Parameters – mostly related to Ceres options
    bool baInverseDepthEnabled_ = true;
    int baMinNumCommonKeypointsObservations_ = 25;

    bool robustCostRefineWithL2_ = true;
    float robustCostThreshold_ = 5.9915;
};
