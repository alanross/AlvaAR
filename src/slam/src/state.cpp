#include "state.hpp"

State::State(double imgWidth, double imgHeight, int frameMaxCellSize)
{
    imgWidth_ = imgWidth;
    imgHeight_ = imgHeight;

    frameMaxCellSize_ = frameMaxCellSize;
    float numCellsW = ceil((float) imgWidth_ / (float) frameMaxCellSize_);
    float numCellsH = ceil((float) imgHeight_ / (float) frameMaxCellSize_);
    frameMaxNumKeypoints_ = (int) (numCellsW * numCellsH);
}

void State::reset()
{
    slamReadyForInit_ = false;
    slamResetRequested_ = false;
}
