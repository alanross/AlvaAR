#include "state.hpp"

State::State(double width, double height, double fx, double fy, double cx, double cy, double k1, double k2, double p1, double p2)
{
    imgWidth_ = width;
    imgHeight_ = height;

    fxl_ = fx;
    fyl_ = fy;
    cxl_ = cx;
    cyl_ = cy;

    k1l_ = k1;
    k2l_ = k2;
    p1l_ = p1;
    p2l_ = p2;

    float numCellsW = ceil((float) imgWidth_ / (float) frame_max_cell_size_);
    float numCellsH = ceil((float) imgHeight_ / (float) frame_max_cell_size_);
    frame_max_num_kps_ = (int) (numCellsW * numCellsH);
}

void State::reset()
{
    localBAActive_ = false;
    loopClosureActive_ = false;
    slamReadyForInit_ = false;
    slamResetRequested_ = false;
}
