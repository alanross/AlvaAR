#pragma once

#include <queue>
#include <deque>

#include "map_manager.hpp"
#include "optimizer.hpp"

class Estimator
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Estimator(std::shared_ptr<State> state, std::shared_ptr<MapManager> mapManager) : state_(state), mapManager_(mapManager), optimizer_(new Optimizer(state_, mapManager_))
    {
    }

    void reset();

    void applyLocalBA();

    void mapFiltering();

    bool getNewKeyframe();

    void addNewKeyframe(const std::shared_ptr<Frame> &keyframe);

    void timedOperationStart();

    bool timedOperationHasTimedOut();

    std::shared_ptr<State> state_;
    std::shared_ptr<MapManager> mapManager_;
    std::shared_ptr<Frame> newKeyframe_;
    std::unique_ptr<Optimizer> optimizer_;
    std::queue<std::shared_ptr<Frame>> keyframeQueue_;

    std::chrono::high_resolution_clock::time_point timedOperationStartTime_;
};