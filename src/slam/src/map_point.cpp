#include "map_point.hpp"

MapPoint::MapPoint(const int mapPointId, const int keyframeId, const bool observed)
        : mapPointId_(mapPointId), isObserved_(observed), keyframeId_(keyframeId), invDepth_(-1.)
{
    observedKeyframeIds_.insert(keyframeId);
    is3d_ = false;
    point3d_.setZero();
}

MapPoint::MapPoint(const int mapPointId, const int keyframeId, const cv::Mat &descriptor, const bool observed)
        : mapPointId_(mapPointId), isObserved_(observed), keyframeId_(keyframeId), invDepth_(-1.)
{
    observedKeyframeIds_.insert(keyframeId);

    mapKeyframeDescriptors_.emplace(keyframeId, descriptor);
    mapDescriptorsDist_.emplace(keyframeId, 0.);
    desc_ = mapKeyframeDescriptors_.at(keyframeId);

    is3d_ = false;
    point3d_.setZero();
}

MapPoint::MapPoint(const int mapPointId, const int keyframeId, const cv::Scalar &color, const bool observed)
        : mapPointId_(mapPointId), isObserved_(observed), keyframeId_(keyframeId), invDepth_(-1.), color_(color)
{
    observedKeyframeIds_.insert(keyframeId);
    is3d_ = false;
    point3d_.setZero();
}

MapPoint::MapPoint(const int mapPointId, const int keyframeId, const cv::Mat &descriptor, const cv::Scalar &color, const bool observed)
        : mapPointId_(mapPointId), isObserved_(observed), keyframeId_(keyframeId), invDepth_(-1.), color_(color)
{
    observedKeyframeIds_.insert(keyframeId);

    mapKeyframeDescriptors_.emplace(keyframeId, descriptor);
    mapDescriptorsDist_.emplace(keyframeId, 0.);
    desc_ = mapKeyframeDescriptors_.at(keyframeId);

    is3d_ = false;
    point3d_.setZero();
}

void MapPoint::setPoint(const Eigen::Vector3d &point3d, const double keyframeAnchorInvDepth)
{
    point3d_ = point3d;
    is3d_ = true;
    if (keyframeAnchorInvDepth >= 0.)
    {
        invDepth_ = keyframeAnchorInvDepth;
    }
}

Eigen::Vector3d MapPoint::getPoint() const
{
    return point3d_;
}

std::set<int> MapPoint::getObservedKeyframeIds() const
{
    return observedKeyframeIds_;
}

void MapPoint::addObservedKeyframeId(const int keyframeId)
{
    observedKeyframeIds_.insert(keyframeId);
}

void MapPoint::removeObservedKeyframeId(const int keyframeId)
{
    if (!observedKeyframeIds_.count(keyframeId))
    {
        return;
    }

    // First remove the related id
    observedKeyframeIds_.erase(keyframeId);

    if (observedKeyframeIds_.empty())
    {
        desc_.release();
        mapKeyframeDescriptors_.clear();
        mapDescriptorsDist_.clear();
        return;
    }

    // Set new keyframe anchor if removed
    if (keyframeId == keyframeId_)
    {
        keyframeId_ = *observedKeyframeIds_.begin();
    }

    // Find most representative one
    float minDist = desc_.cols * 8.;
    int minId = -1;

    auto itDescriptors = mapKeyframeDescriptors_.find(keyframeId);

    if (itDescriptors != mapKeyframeDescriptors_.end())
    {
        for (const auto &descriptor: mapKeyframeDescriptors_)
        {
            if (descriptor.first != keyframeId)
            {
                float dist = cv::norm(itDescriptors->second, descriptor.second, cv::NORM_HAMMING);
                float &descDist = mapDescriptorsDist_.find(descriptor.first)->second;
                descDist -= dist;

                // Get the lowest one
                if (descDist < minDist)
                {
                    minDist = descDist;
                    minId = descriptor.first;
                }
            }
        }

        itDescriptors->second.release();
        mapKeyframeDescriptors_.erase(keyframeId);
        mapDescriptorsDist_.erase(keyframeId);

        // Remove desc / update mean desc
        if (minId > 0)
        {
            desc_ = mapKeyframeDescriptors_.at(minId);
        }
    }
}

void MapPoint::addDesc(const int keyframeId, const cv::Mat &descriptor)
{
    auto iterator = mapKeyframeDescriptors_.find(keyframeId);
    if (iterator != mapKeyframeDescriptors_.end())
    {
        return;
    }

    // First add the desc and init its distance score
    mapKeyframeDescriptors_.emplace(keyframeId, descriptor);

    mapDescriptorsDist_.emplace(keyframeId, 0);
    float &newDescriptorDist = mapDescriptorsDist_.find(keyframeId)->second;

    if (mapKeyframeDescriptors_.size() == 1)
    {
        desc_ = descriptor;
        return;
    }

    // Then figure out the most representative one (we could also use the last one to save time)
    float minDist = desc_.cols * 8.;
    int minId = -1;

    // Then update the distance scores for all desc
    for (const auto &kf_d: mapKeyframeDescriptors_)
    {
        float dist = cv::norm(descriptor, kf_d.second, cv::NORM_HAMMING);

        // Update previous desc
        mapDescriptorsDist_.at(kf_d.first) += dist;

        // Get the lowest one
        if (dist < minDist)
        {
            minDist = dist;
            minId = kf_d.first;
        }

        // Update new desc
        newDescriptorDist += dist;
    }

    // Get the lowest one
    if (newDescriptorDist < minDist)
    {
        minId = keyframeId;
    }

    desc_ = mapKeyframeDescriptors_.at(minId);
}

bool MapPoint::isBad()
{
    // Set as bad 3D map points who are observed by 2 keyframe or less and not observed by current frame
    if (observedKeyframeIds_.size() < 2)
    {
        if (!isObserved_ && is3d_)
        {
            is3d_ = false;
            return true;
        }
    }

    if (observedKeyframeIds_.size() == 0 && !isObserved_)
    {
        is3d_ = false;
        return true;
    }

    return false;
}

float MapPoint::computeMinDescDist(const MapPoint &mapPoint)
{
    float minDist = 1000.0;

    for (const auto &kf_desc: mapKeyframeDescriptors_)
    {
        for (const auto &kf_desc2: mapPoint.mapKeyframeDescriptors_)
        {
            float dist = cv::norm(kf_desc.second, kf_desc2.second, cv::NORM_HAMMING);

            if (dist < minDist)
            {
                minDist = dist;
            }
        }
    }

    return minDist;
}
