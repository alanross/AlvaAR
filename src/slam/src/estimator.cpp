#include "estimator.hpp"

bool Estimator::getNewKeyframe()
{
    // check if new keyframe is available
    if (keyframeQueue_.empty())
    {
        return false;
    }

    // only processed last received keyframe. Trick covscore if several keyframes were waiting
    std::vector<int> keyframeIds;
    keyframeIds.reserve(keyframeQueue_.size());

    while (keyframeQueue_.size() > 1)
    {
        keyframeQueue_.pop();
        keyframeIds.push_back(newKeyframe_->keyframeId_);
    }

    newKeyframe_ = keyframeQueue_.front();
    keyframeQueue_.pop();

    if (!keyframeIds.empty())
    {
        for (const auto &keyframeId: keyframeIds)
        {
            newKeyframe_->covisibleKeyframeIds_[keyframeId] = newKeyframe_->numKeypoints3d_;
        }
    }

    return true;
}

void Estimator::addNewKeyframe(const std::shared_ptr<Frame> &keyframe)
{
    timedOperationStart();

    keyframeQueue_.push(keyframe);

    if (getNewKeyframe())
    {
        applyLocalBA();
        mapFiltering();
    }
}

void Estimator::applyLocalBA()
{
    int minNumKeyframes = 2;

    if (newKeyframe_->keyframeId_ < minNumKeyframes)
    {
        return;
    }

    if (newKeyframe_->numKeypoints3d_ == 0)
    {
        return;
    }

    optimizer_->localBA(*newKeyframe_, true);
}

void Estimator::mapFiltering()
{
    if (state_->mapKeyframeFilteringRatio_ >= 1.)
    {
        return;
    }

    if (newKeyframe_->keyframeId_ < 20)
    {
        return;
    }

    auto covkf_map = newKeyframe_->getCovisibleKeyframeMap();

    for (auto it = covkf_map.rbegin(); it != covkf_map.rend(); it++)
    {
        int kfid = it->first;

        if (timedOperationHasTimedOut() || kfid == 0)
        {
            break;
        }

        if (kfid >= newKeyframe_->keyframeId_)
        {
            continue;
        }

        auto pkf = mapManager_->getKeyframe(kfid);
        if (pkf == nullptr)
        {
            newKeyframe_->removeCovisibleKeyframe(kfid);
            continue;
        }
        else if ((int) pkf->numKeypoints3d_ < state_->localBAMinNumCommonKeypointsObservations_ / 2)
        {
            mapManager_->removeKeyframe(kfid);
            continue;
        }

        size_t numGoodObservations = 0;
        size_t numTotal = 0;

        for (const auto &kp: pkf->getKeypoints3d())
        {
            auto plm = mapManager_->getMapPoint(kp.keypointId_);

            if (plm == nullptr)
            {
                mapManager_->removeMapPointObs(kp.keypointId_, kfid);
                continue;
            }
            else if (plm->isBad())
            {
                continue;
            }
            else
            {
                size_t nbcokfs = plm->getObservedKeyframeIds().size();
                if (nbcokfs > 4)
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

        float ratio = (float) numGoodObservations / numTotal;

        if (ratio > state_->mapKeyframeFilteringRatio_)
        {
            mapManager_->removeKeyframe(kfid);
        }
    }
}

void Estimator::reset()
{
    std::queue<std::shared_ptr<Frame>> empty;
    std::swap(keyframeQueue_, empty);
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