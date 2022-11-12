#include "state.hpp"

State::State(double imgWidth, double imgHeight )
{
    imgWidth_ = imgWidth;
    imgHeight_ = imgHeight;

    frameMaxCellSize_ = 40;
    claheEnabled_ = false;
    mapKeyframeFilteringRatio = 0.95;
    p3pEnabled_ = true;

    float numCellsW = ceil((float) imgWidth_ / (float) frameMaxCellSize_);
    float numCellsH = ceil((float) imgHeight_ / (float) frameMaxCellSize_);
    frameMaxNumKeypoints_ = (int) (numCellsW * numCellsH);

    std::cout << "- [State]: Config";
    std::cout << ": width: " << imgWidth;
    std::cout << ", height: " << imgHeight;
    std::cout << ", Frame Max Cell Size: " << frameMaxCellSize_;
    std::cout << ", CLAHE Enabled: " << claheEnabled_;
    std::cout << ", Map Keyframe Filtering Ratio: " << mapKeyframeFilteringRatio;
    std::cout << ", P3P Enabled: " << p3pEnabled_ << std::endl;
}

void State::reset()
{
    localBAActive_ = false;
    loopClosureActive_ = false;
    slamReadyForInit_ = false;
    slamResetRequested_ = false;
}
