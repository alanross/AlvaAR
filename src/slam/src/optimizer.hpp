#pragma once

#include <iomanip>
#include <deque>
#include <fstream>
#include "map_manager.hpp"

class Optimizer
{

public:
    Optimizer(std::shared_ptr<State> state, std::shared_ptr<MapManager> mapManager) : state_(state), mapManager_(mapManager)
    {
    }

    void localBA(Frame &newFrame);

private:
    std::shared_ptr<State> state_;
    std::shared_ptr<MapManager> mapManager_;
};