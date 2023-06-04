#include <opencv2/highgui.hpp>
#include "multi_view_geometry.hpp"
#include "map_manager.hpp"

MapManager::MapManager(std::shared_ptr<State> state, std::shared_ptr<Frame> frame, std::shared_ptr<FeatureExtractor> featureExtractor)
        : numMapPointIds_(0), numKeyframeIds_(0), numMapPoints_(0), numKeyframes_(0), state_(state), featureExtractor_(featureExtractor), currFrame_(frame)
{
    pointCloud_.clear();
    pointCloud_.reserve(1e5);
}

void MapManager::createKeyframe(const cv::Mat &image, const cv::Mat &imageRaw)
{
    // Prepare Frame to become a keyframe (Update observations between map points / keyframes)
    prepareFrame();

    // Detect in image and describe in imageRaw
    extractKeypoints(image, imageRaw);

    // Add keyframe to the map
    addKeyframe();
}

void MapManager::prepareFrame()
{
    // Update new keyframe id
    currFrame_->keyframeId_ = numKeyframeIds_;

    // Filter if too many keypoints
    if ((int) currFrame_->numKeypoints_ > state_->frameMaxNumKeypoints_)
    {
        for (const auto &keypointIds: currFrame_->gridKeypointsIds_)
        {
            if (keypointIds.size() > 2)
            {
                int mapPointIdToBeRemoved = -1;
                size_t minNumObs = std::numeric_limits<size_t>::max();

                for (const auto &lmid: keypointIds)
                {
                    auto iterator = mapMapPoints_.find(lmid);

                    if (iterator != mapMapPoints_.end())
                    {
                        size_t numObs = iterator->second->getObservedKeyframeIds().size();

                        if (numObs < minNumObs)
                        {
                            mapPointIdToBeRemoved = lmid;
                            minNumObs = numObs;
                        }
                    }
                    else
                    {
                        removeObsFromCurrFrameById(lmid);
                        break;
                    }
                }
                if (mapPointIdToBeRemoved >= 0)
                {
                    removeObsFromCurrFrameById(mapPointIdToBeRemoved);
                }
            }
        }
    }

    for (const auto &kp: currFrame_->getKeypoints())
    {
        // Get the related map points
        auto iterator = mapMapPoints_.find(kp.keypointId_);

        if (iterator == mapMapPoints_.end())
        {
            removeObsFromCurrFrameById(kp.keypointId_);
            continue;
        }

        // Relate new keyframe id to the map point
        iterator->second->addObservedKeyframeId(numKeyframeIds_);
    }
}

void MapManager::updateFrameCovisibility(Frame &frame)
{
    // Update the map points and the covisilbe graph between keyframes
    std::map<int, int> mapCovisibleKeyframes;
    std::unordered_set<int> localMapIds;

    for (const auto &kp: frame.getKeypoints())
    {
        // Get the related map point
        auto iterator = mapMapPoints_.find(kp.keypointId_);

        if (iterator == mapMapPoints_.end())
        {
            removeMapPointObs(kp.keypointId_, frame.keyframeId_);
            removeObsFromCurrFrameById(kp.keypointId_);
            continue;
        }

        // Get the set of keyframes observing this keyframe to update covisible keyframes
        for (const auto &kfid: iterator->second->getObservedKeyframeIds())
        {
            if (kfid != frame.keyframeId_)
            {
                auto it = mapCovisibleKeyframes.find(kfid);
                if (it != mapCovisibleKeyframes.end())
                {
                    it->second += 1;
                }
                else
                {
                    mapCovisibleKeyframes.emplace(kfid, 1);
                }
            }
        }
    }

    // Update covisibility for covisible keyframes
    std::set<int> set_badkfids;
    for (const auto &covisibleKeyframeId: mapCovisibleKeyframes)
    {
        int keyframeId = covisibleKeyframeId.first;
        int covScore = covisibleKeyframeId.second;

        auto iterator = mapKeyframes_.find(keyframeId);
        if (iterator != mapKeyframes_.end())
        {
            // Will emplace or update covisiblity
            iterator->second->covisibleKeyframeIds_[frame.keyframeId_] = covScore;

            // Set the unobserved local map for future tracking
            for (const auto &kp: iterator->second->getKeypoints3d())
            {
                if (!frame.isObservingKeypoint(kp.keypointId_))
                {
                    localMapIds.insert(kp.keypointId_);
                }
            }
        }
        else
        {
            set_badkfids.insert(keyframeId);
        }
    }

    for (const auto &keyframeId: set_badkfids)
    {
        mapCovisibleKeyframes.erase(keyframeId);
    }

    // Update the set of covisible keyframes
    frame.covisibleKeyframeIds_.swap(mapCovisibleKeyframes);

    // Update local map of unobserved map points
    if (localMapIds.size() > 0.5 * frame.localMapPointIds_.size())
    {
        frame.localMapPointIds_.swap(localMapIds);
    }
    else
    {
        frame.localMapPointIds_.insert(localMapIds.begin(), localMapIds.end());
    }
}

void MapManager::addKeypointsToFrame(const cv::Mat &image, const std::vector<cv::Point2f> &points, const std::vector<cv::Mat> &descriptors, Frame &frame)
{
    // Add keypoints + create landmarks
    size_t numPoints = points.size();
    for (size_t i = 0; i < numPoints; i++)
    {
        if (!descriptors.at(i).empty())
        {
            // Add keypoint to current frame
            frame.addKeypoint(points.at(i), numMapPointIds_, descriptors.at(i));

            // Create landmark with same id
            cv::Scalar pixel = image.at<uchar>(points.at(i).y, points.at(i).x);
            addMapPoint(descriptors.at(i), pixel);
        }
        else
        {
            // Add keypoint to current frame
            frame.addKeypoint(points.at(i), numMapPointIds_);

            // Create landmark with same id
            cv::Scalar pixel = image.at<uchar>(points.at(i).y, points.at(i).x);
            addMapPoint(pixel);
        }
    }
}

void MapManager::extractKeypoints(const cv::Mat &image, const cv::Mat &imageRaw)
{
    std::vector<Keypoint> keypoints = currFrame_->getKeypoints();

    std::vector<cv::Point2f> points;

    for (auto &kp: keypoints)
    {
        points.push_back(kp.px_);
    }

    describeKeypoints(imageRaw, keypoints, points);

    int numToDetect = state_->frameMaxNumKeypoints_ - currFrame_->numOccupiedCells_;

    if (numToDetect > 0)
    {
        // Detect kps in the provided images using the cur kps and img roi to set a mask
        std::vector<cv::Point2f> newPoints;

        newPoints = featureExtractor_->detectFeaturePoints(image, state_->frameMaxCellSize_, points, currFrame_->cameraCalibration_->roi_rect_);

        if (!newPoints.empty())
        {
            std::vector<cv::Mat> vdescs;
            vdescs = featureExtractor_->describeFeaturePoints(imageRaw, newPoints);
            addKeypointsToFrame(image, newPoints, vdescs, *currFrame_);
        }
    }
}

void MapManager::describeKeypoints(const cv::Mat &image, const std::vector<Keypoint> &keypoints, const std::vector<cv::Point2f> &points)
{
    size_t numKeypoints = keypoints.size();
    std::vector<cv::Mat> descriptors;

    descriptors = featureExtractor_->describeFeaturePoints(image, points);

    assert(keypoints.size() == descriptors.size());

    for (size_t i = 0; i < numKeypoints; i++)
    {
        if (!descriptors.at(i).empty())
        {
            currFrame_->updateKeypointDesc(keypoints.at(i).keypointId_, descriptors.at(i));
            mapMapPoints_.at(keypoints.at(i).keypointId_)->addDesc(currFrame_->keyframeId_, descriptors.at(i));
        }
    }
}

void MapManager::addKeyframe()
{
    // Create a copy of current frame shared_ptr for creating an independent keyframe to add to the map
    std::shared_ptr<Frame> keyframe = std::allocate_shared<Frame>(Eigen::aligned_allocator<Frame>(), *currFrame_);

    // Add keyframe to the unordered map and update id/nb
    mapKeyframes_.emplace(numKeyframeIds_, keyframe);
    numKeyframes_++;
    numKeyframeIds_++;
}

void MapManager::addMapPoint(const cv::Scalar &color)
{
    // Create a new map point with a unique lmid and a keyframe id obs
    std::shared_ptr<MapPoint> mapPoint = std::allocate_shared<MapPoint>(Eigen::aligned_allocator<MapPoint>(), numMapPointIds_, numKeyframeIds_, color);

    // Add new map point to the map and update id/nb
    mapMapPoints_.emplace(numMapPointIds_, mapPoint);
    numMapPointIds_++;
    numMapPoints_++;

    Point3D point;
    point = Point3D();

    if (mapPoint->isObserved_)
    {
        point.r = 255;
        point.g = 0;
        point.b = 0;
    }
    else
    {
        point.r = mapPoint->color_[0];
        point.g = mapPoint->color_[0];
        point.b = mapPoint->color_[0];
    }

    point.x = 0.0;
    point.y = 0.0;
    point.z = 0.0;

    pointCloud_.push_back(point);
}

void MapManager::addMapPoint(const cv::Mat &desc, const cv::Scalar &color)
{
    // Create a new map point with a unique id and a keyframe id obs
    std::shared_ptr<MapPoint> mapPoint = std::allocate_shared<MapPoint>(Eigen::aligned_allocator<MapPoint>(), numMapPointIds_, numKeyframeIds_, desc, color);

    // Add new map point to the map and update id/nb
    mapMapPoints_.emplace(numMapPointIds_, mapPoint);
    numMapPointIds_++;
    numMapPoints_++;

    Point3D point;
    point = Point3D();

    if (mapPoint->isObserved_)
    {
        point.r = 255;
        point.g = 0;
        point.b = 0;
    }
    else
    {
        point.r = mapPoint->color_[0];
        point.g = mapPoint->color_[0];
        point.b = mapPoint->color_[0];
    }

    point.x = 0.0;
    point.y = 0.0;
    point.z = 0.0;

    pointCloud_.push_back(point);
}

std::shared_ptr<Frame> MapManager::getKeyframe(const int KeyframeId) const
{
    auto it = mapKeyframes_.find(KeyframeId);
    if (it == mapKeyframes_.end())
    {
        return nullptr;
    }
    return it->second;
}

std::shared_ptr<MapPoint> MapManager::getMapPoint(const int mapPointId) const
{
    auto it = mapMapPoints_.find(mapPointId);
    if (it == mapMapPoints_.end())
    {
        return nullptr;
    }
    return it->second;
}

std::vector<Eigen::Vector3d> MapManager::getCurrentFrameMapPoints() const
{
    std::vector<Eigen::Vector3d> mapPoints;

    auto it = mapMapPoints_.begin();

    while (it != mapMapPoints_.end())
    {
        if (it->second->isObserved_ && it->second->is3d_)
        {
            mapPoints.push_back(it->second->point3d_);
        }

        it++;
    }

    return mapPoints;
}

void MapManager::updateMapPoint(const int mapPointId, const Eigen::Vector3d &wpt, const double keyframeAnchorInvDepth)
{
    auto mapPointIterator = mapMapPoints_.find(mapPointId);

    if (mapPointIterator == mapMapPoints_.end())
    {
        return;
    }

    if (mapPointIterator->second == nullptr)
    {
        return;
    }

    // if map point 2d -> 3d => notify keyframes
    if (!mapPointIterator->second->is3d_)
    {
        for (const auto &keyframeId: mapPointIterator->second->getObservedKeyframeIds())
        {
            auto keyframes = mapKeyframes_.find(keyframeId);
            if (keyframes != mapKeyframes_.end())
            {
                keyframes->second->turnKeypoint3d(mapPointId);
            }
            else
            {
                mapPointIterator->second->removeObservedKeyframeId(keyframeId);
            }
        }

        if (mapPointIterator->second->isObserved_)
        {
            currFrame_->turnKeypoint3d(mapPointId);
        }
    }

    // Update map point world position
    if (keyframeAnchorInvDepth >= 0.)
    {
        mapPointIterator->second->setPoint(wpt, keyframeAnchorInvDepth);
    }
    else
    {
        mapPointIterator->second->setPoint(wpt);
    }

    Point3D point;
    point = Point3D();

    if (mapPointIterator->second->isObserved_)
    {
        point.r = 255;
        point.g = 0;
        point.b = 0;
    }
    else
    {
        point.r = mapPointIterator->second->color_[0];
        point.g = mapPointIterator->second->color_[0];
        point.b = mapPointIterator->second->color_[0];
    }

    point.x = wpt.x();
    point.y = wpt.y();
    point.z = wpt.z();

    pointCloud_.at(mapPointId) = point;
}

void MapManager::mergeMapPoints(const int prvMapPointId, const int newMapPointId)
{
    // 1. Get Kf obs + descs from prev map point
    // 2. Remove prev map point
    // 3. Update new map point and related keyframe / cur Frame

    // Get prev map point to merge into new map point

    auto prvLmIt = mapMapPoints_.find(prvMapPointId);
    auto newLmIt = mapMapPoints_.find(newMapPointId);

    if (prvLmIt == mapMapPoints_.end())
    {
        return;
    }
    else if (newLmIt == mapMapPoints_.end())
    {
        return;
    }
    else if (!newLmIt->second->is3d_)
    {
        return;
    }

    // 1. Get Kf obs + descs from prev map point
    std::set<int> nextKfIds = newLmIt->second->getObservedKeyframeIds();
    std::set<int> prevKfIds = prvLmIt->second->getObservedKeyframeIds();
    std::unordered_map<int, cv::Mat> map_prev_kf_desc_ = prvLmIt->second->mapKeyframeDescriptors_;

    // 3. Update new map point and related keyframe / cur Frame
    for (const auto &pkfid: prevKfIds)
    {
        // Get prev keyframe and update keypoint
        auto pkfit = mapKeyframes_.find(pkfid);
        if (pkfit != mapKeyframes_.end())
        {
            if (pkfit->second->updateKeypointId(prvMapPointId, newMapPointId, newLmIt->second->is3d_))
            {
                newLmIt->second->addObservedKeyframeId(pkfid);
                for (const auto &nkfid: nextKfIds)
                {
                    auto pcokfit = mapKeyframes_.find(nkfid);
                    if (pcokfit != mapKeyframes_.end())
                    {
                        pkfit->second->addCovisibleKeyframe(nkfid);
                        pcokfit->second->addCovisibleKeyframe(pkfid);
                    }
                }
            }
        }
    }

    for (const auto &kfid_desc: map_prev_kf_desc_)
    {
        newLmIt->second->addDesc(kfid_desc.first, kfid_desc.second);
    }

    // Turn new map point observed by curr frame if prev map point
    // was + update curr frame's keypoint reference to new map point
    if (currFrame_->isObservingKeypoint(prvMapPointId))
    {
        if (currFrame_->updateKeypointId(prvMapPointId, newMapPointId, newLmIt->second->is3d_))
        {
            setMapPointObs(newMapPointId);
        }
    }

    if (prvLmIt->second->is3d_)
    {
        numMapPoints_--;
    }

    // Erase map point and update nb map points
    mapMapPoints_.erase(prvLmIt);

    Point3D point;
    point = Point3D();
    point.r = 0;
    point.g = 0;
    point.b = 0;
    point.x = 0.0;
    point.y = 0.0;
    point.z = 0.0;

    pointCloud_[prvMapPointId] = point;
}

void MapManager::removeKeyframe(const int keyframeId)
{
    // Get keyframe to remove
    auto pkfit = mapKeyframes_.find(keyframeId);

    // Skip if keyframe does not exist
    if (pkfit == mapKeyframes_.end())
    {
        return;
    }

    // Remove the keyframe obs from all observed map point
    for (const auto &kp: pkfit->second->getKeypoints())
    {
        // Get map point and remove keyframe obs
        auto plmit = mapMapPoints_.find(kp.keypointId_);
        if (plmit == mapMapPoints_.end())
        {
            continue;
        }
        plmit->second->removeObservedKeyframeId(keyframeId);
    }

    for (const auto &kfid_cov: pkfit->second->getCovisibleKeyframeMap())
    {
        auto pcokfit = mapKeyframes_.find(kfid_cov.first);
        if (pcokfit != mapKeyframes_.end())
        {
            pcokfit->second->removeCovisibleKeyframe(keyframeId);
        }
    }

    // Remove keyframe and update nb keyframes
    mapKeyframes_.erase(pkfit);
    numKeyframes_--;

    if (state_->debug_)
    {
        std::cout << "- [Map-Manager]: Remove keyframe #" << keyframeId << std::endl;
    }
}

void MapManager::removeMapPoint(const int mapPointId)
{
    // Get related map point
    auto plmit = mapMapPoints_.find(mapPointId);

    // Skip if map point does not exist
    if (plmit != mapMapPoints_.end())
    {
        // Remove all observations from keyframes
        for (const auto &kfid: plmit->second->getObservedKeyframeIds())
        {
            auto pkfit = mapKeyframes_.find(kfid);
            if (pkfit == mapKeyframes_.end())
            {
                continue;
            }

            pkfit->second->removeKeypointById(mapPointId);

            for (const auto &cokfid: plmit->second->getObservedKeyframeIds())
            {
                if (cokfid != kfid)
                {
                    pkfit->second->decreaseCovisibleKeyframe(cokfid);
                }
            }
        }

        // If obs in cur Frame, remove cur obs
        if (plmit->second->isObserved_)
        {
            currFrame_->removeKeypointById(mapPointId);
        }

        if (plmit->second->is3d_)
        {
            numMapPoints_--;
        }

        // Erase map point and update nb map points
        mapMapPoints_.erase(plmit);
    }

    Point3D point;
    point = Point3D();
    point.r = 0;
    point.g = 0;
    point.b = 0;
    point.x = 0.0;
    point.y = 0.0;
    point.z = 0.0;

    pointCloud_.at(mapPointId) = point;
}

void MapManager::removeMapPointObs(const int mapPointId, const int keyframeId)
{
    // Remove map point obs from keyframe
    auto pkfit = mapKeyframes_.find(keyframeId);
    if (pkfit != mapKeyframes_.end())
    {
        pkfit->second->removeKeypointById(mapPointId);
    }

    // Remove keyframe obs from map point
    auto plmit = mapMapPoints_.find(mapPointId);

    // Skip if map point does not exist
    if (plmit == mapMapPoints_.end())
    {
        return;
    }
    plmit->second->removeObservedKeyframeId(keyframeId);

    if (pkfit != mapKeyframes_.end())
    {
        for (const auto &cokfid: plmit->second->getObservedKeyframeIds())
        {
            auto pcokfit = mapKeyframes_.find(cokfid);
            if (pcokfit != mapKeyframes_.end())
            {
                pkfit->second->decreaseCovisibleKeyframe(cokfid);
                pcokfit->second->decreaseCovisibleKeyframe(keyframeId);
            }
        }
    }
}

void MapManager::removeObsFromCurrFrameById(const int mapPointId)
{
    // Remove cur obs
    currFrame_->removeKeypointById(mapPointId);

    // Set map point as not obs
    auto iterator = mapMapPoints_.find(mapPointId);

    Point3D point;

    // Skip if map point does not exist
    if (iterator == mapMapPoints_.end())
    {
        // Set the map point at origin
        pointCloud_.at(mapPointId) = point;
        return;
    }

    iterator->second->isObserved_ = false;

    // Update map point color
    point = Point3D();
    point.r = iterator->second->color_[0];
    point.g = iterator->second->color_[0];
    point.b = iterator->second->color_[0];
    point.x = pointCloud_.at(mapPointId).x;
    point.y = pointCloud_.at(mapPointId).y;
    point.z = pointCloud_.at(mapPointId).z;

    pointCloud_.at(mapPointId) = point;
}

bool MapManager::setMapPointObs(const int mapPointId)
{
    auto iterator = mapMapPoints_.find(mapPointId);

    Point3D point;

    // Skip if map point does not exist
    if (iterator == mapMapPoints_.end())
    {
        // Set the map point at origin
        pointCloud_.at(mapPointId) = point;
        return false;
    }

    iterator->second->isObserved_ = true;

    // Update map point color
    point = Point3D();
    point.r = 200;
    point.g = 0;
    point.b = 0;
    point.x = pointCloud_.at(mapPointId).x;
    point.y = pointCloud_.at(mapPointId).y;
    point.z = pointCloud_.at(mapPointId).z;

    pointCloud_.at(mapPointId) = point;

    return true;
}

void MapManager::reset()
{
    numMapPointIds_ = 0;
    numKeyframeIds_ = 0;
    numMapPoints_ = 0;
    numKeyframes_ = 0;

    mapKeyframes_.clear();
    mapMapPoints_.clear();

    pointCloud_.clear();
}
