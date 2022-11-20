#include "estimator.hpp"

void Estimator::addNewKeyframe(const std::shared_ptr<Frame> &keyframe)
{
    timedOperationStart();

    newKeyframe_ = keyframe;

    applyLocalBA();
    applyMapFiltering();
}

void Estimator::applyLocalBA()
{
    if (newKeyframe_->keyframeId_ < 2 || newKeyframe_->numKeypoints3d_ == 0)
    {
        return;
    }

    optimizer_->localBA(*newKeyframe_, true);
}

void Estimator::applyMapFiltering()
{
    if (state_->mapKeyframeFilteringRatio_ >= 1.0 || newKeyframe_->keyframeId_ < 20)
    {
        return;
    }

    auto covisibleKeyframeMap = newKeyframe_->getCovisibleKeyframeMap();

    for (auto it = covisibleKeyframeMap.rbegin(); it != covisibleKeyframeMap.rend(); it++)
    {
        int keyframeId = it->first;

        if (timedOperationHasTimedOut() || keyframeId == 0)
        {
            break;
        }

        if (keyframeId >= newKeyframe_->keyframeId_)
        {
            continue;
        }

        auto keyframe = mapManager_->getKeyframe(keyframeId);
        if (keyframe == nullptr)
        {
            newKeyframe_->removeCovisibleKeyframe(keyframeId);
            continue;
        }
        else if ((int) keyframe->numKeypoints3d_ < state_->localBAMinNumCommonKeypointsObservations_ / 2)
        {
            mapManager_->removeKeyframe(keyframeId);
            continue;
        }

        size_t numGoodObservations = 0;
        size_t numTotal = 0;

        for (const auto &kp: keyframe->getKeypoints3d())
        {
            auto mapPoint = mapManager_->getMapPoint(kp.keypointId_);

            if (mapPoint == nullptr)
            {
                mapManager_->removeMapPointObs(kp.keypointId_, keyframeId);
                continue;
            }
            else if (mapPoint->isBad())
            {
                continue;
            }
            else
            {
                size_t numObservedKeyframeIds = mapPoint->getObservedKeyframeIds().size();

                if (numObservedKeyframeIds > 4)
                {
                    numGoodObservations++;
                }
            }

            numTotal++;

            if (timedOperationHasTimedOut())
            {
                break;
            }
        }

        float ratio = (float) numGoodObservations / (float) numTotal;

        if (ratio > state_->mapKeyframeFilteringRatio_)
        {
            mapManager_->removeKeyframe(keyframeId);
        }
    }
}

void Estimator::timedOperationStart()
{
    timedOperationStartTime_ = std::chrono::high_resolution_clock::now();
}

bool Estimator::timedOperationHasTimedOut()
{
    auto now = std::chrono::high_resolution_clock::now();
    auto dif = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - timedOperationStartTime_).count();

    return (dif > 8);
}