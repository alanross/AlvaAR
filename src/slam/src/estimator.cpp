#include "estimator.hpp"

bool Estimator::getNewKeyframe()
{
    // check if new keyframe is available
    if (keyframeQueue_.empty())
    {
        newKeyframeAvailable_ = false;
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

    newKeyframeAvailable_ = false;

    return true;
}

void Estimator::addNewKeyframe(const std::shared_ptr<Frame> &keyframe)
{
    keyframeQueue_.push(keyframe);
    newKeyframeAvailable_ = true;

    if (getNewKeyframe())
    {
        applyLocalBA();
        mapFiltering();
    }
}

void Estimator::applyLocalBA()
{
    int nmincstkfs = 2;

    if (newKeyframe_->keyframeId_ < nmincstkfs)
    {
        return;
    }

    if (newKeyframe_->numKeypoints3d_ == 0)
    {
        return;
    }

    // signal that Estimator is performing BA
    state_->localBAActive_ = true;

    optimizer_->localBA(*newKeyframe_, true);

    // signal that Estimator is stopping BA
    state_->localBAActive_ = false;
}

void Estimator::mapFiltering()
{
    if (state_->mapKeyframeFilteringRatio >= 1.)
    {
        return;
    }

    if (newKeyframe_->keyframeId_ < 20 || state_->loopClosureActive_)
    {
        return;
    }

    auto covkf_map = newKeyframe_->getCovisibleKeyframeMap();

    for (auto it = covkf_map.rbegin(); it != covkf_map.rend(); it++)
    {
        int kfid = it->first;

        if (newKeyframeAvailable_ || kfid == 0)
        {
            break;
        }

        if (kfid >= newKeyframe_->keyframeId_)
        {
            continue;
        }

        // only useful if loop closing enabled
        if (state_->loopClosureKeyframeId_ == kfid)
        {
            continue;
        }

        auto pkf = mapManager_->getKeyframe(kfid);
        if (pkf == nullptr)
        {
            newKeyframe_->removeCovisibleKeyframe(kfid);
            continue;
        }
        else if ((int) pkf->numKeypoints3d_ < state_->localBAMinNumCommonKeypointsObservations / 2)
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

            if (newKeyframeAvailable_)
            {
                break;
            }
        }

        float ratio = (float) numGoodObservations / numTotal;

        if (ratio > state_->mapKeyframeFilteringRatio)
        {
            // only useful if loop closing enabled
            if (state_->loopClosureKeyframeId_ == kfid)
            {
                continue;
            }

            mapManager_->removeKeyframe(kfid);
        }
    }
}

void Estimator::reset()
{
    newKeyframeAvailable_ = false;

    std::queue<std::shared_ptr<Frame>> empty;
    std::swap(keyframeQueue_, empty);
}