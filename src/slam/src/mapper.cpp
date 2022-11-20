#include "mapper.hpp"

#include <memory>

Mapper::Mapper(std::shared_ptr<State> state, std::shared_ptr<MapManager> mapManager, std::shared_ptr<Frame> frame)
        : state_(state), mapManager_(mapManager), currFrame_(frame), optimizer_(new Optimizer(state_, mapManager_))
{
}

void Mapper::addNewKeyframe(const Keyframe &keyframe)
{
    timedOperationStart();

    // Get new kf ptr
    std::shared_ptr<Frame> newKeyframe = mapManager_->getKeyframe(keyframe.keyframeId_);
    assert(newKeyframe);

    // Triangulate temporal
    if (newKeyframe->numKeypoints2d_ > 0 && newKeyframe->keyframeId_ > 0)
    {
        triangulateTemporal(*newKeyframe);
    }

    // check if reset is required
    if (state_->slamReadyForInit_)
    {
        if (keyframe.keyframeId_ == 1 && newKeyframe->numKeypoints3d_ < 30)
        {
            std::cout << "- [Mapper]: Reset Requested - Bad initialization detected! " << std::endl;
            state_->slamResetRequested_ = true;
            return;
        }
        else if (keyframe.keyframeId_ < 10 && newKeyframe->numKeypoints3d_ < 3)
        {
            std::cout << "- [Mapper]: Reset Requested - Num 3D kps:" << newKeyframe->numKeypoints3d_ << std::endl;
            state_->slamResetRequested_ = true;
            return;
        }
    }

    // Update the map points and the covisible graph between keyframes
    mapManager_->updateFrameCovisibility(*newKeyframe);

    // Dirty but useful for visualization
    currFrame_->covisibleKeyframeIds_ = newKeyframe->covisibleKeyframeIds_;

    if (keyframe.keyframeId_ > 0 && !timedOperationHasTimedOut())
    {
        matchingToLocalMap(*newKeyframe);
    }

    // do bundle adjustment and map filtering
    optimize(newKeyframe);
}

void Mapper::optimize(const std::shared_ptr<Frame> &keyframe)
{
    // removing this will cause time of optimize to be counted as if being part of processNewKeyframe
    timedOperationStart();

    // apply local BA
    if (keyframe->keyframeId_ >= 2 && keyframe->numKeypoints3d_ != 0)
    {
        optimizer_->localBA(*keyframe, true);
    }

    // apply map filtering
    if (state_->mapKeyframeFilteringRatio_ < 1.0 && keyframe->keyframeId_ >= 20)
    {
        auto covisibleKeyframeMap = keyframe->getCovisibleKeyframeMap();

        for (auto it = covisibleKeyframeMap.rbegin(); it != covisibleKeyframeMap.rend(); it++)
        {
            int keyframeId = it->first;

            if (timedOperationHasTimedOut() || keyframeId == 0)
            {
                break;
            }

            if (keyframeId >= keyframe->keyframeId_)
            {
                continue;
            }

            auto keyframe = mapManager_->getKeyframe(keyframeId);
            if (keyframe == nullptr)
            {
                keyframe->removeCovisibleKeyframe(keyframeId);
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
}

void Mapper::triangulateTemporal(Frame &frame)
{
    // Get new keyframe kps / pose
    std::vector<Keypoint> keypoints = frame.getKeypoints2d();

    Sophus::SE3d Twcj = frame.getTwc();

    if (keypoints.empty())
    {
        if (state_->debug_)
        {
            std::cout << "\n \t >>> No kps to temporal triangulate...\n";
        }
        return;
    }

    // Setup triangulation for OpenGV-based mapping
    size_t numKeypoints = keypoints.size();

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > lBvs, rBvs;
    lBvs.reserve(numKeypoints);
    rBvs.reserve(numKeypoints);

    // Init a keyframe object that will point to the prev keyframe to use for triangulation
    std::shared_ptr<Frame> keyframe;
    keyframe = std::make_shared<Frame>();
    keyframe->keyframeId_ = -1;

    // Relative motions between new keyframe and prev. keyframes
    int relKeyframeId = -1;
    Sophus::SE3d Tcicj;
    Sophus::SE3d Tcjci;
    Eigen::Matrix3d Rcicj;

    // New 3D map points projections
    cv::Point2f lPxProj;
    cv::Point2f rPxProj;
    float lDist;
    float rDist;
    Eigen::Vector3d lPoint;
    Eigen::Vector3d rPoint;
    Eigen::Vector3d wpt;

    int good = 0;
    int candidates = 0;

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > wPoints;
    std::vector<int> mapPointIds;
    wPoints.reserve(numKeypoints);
    mapPointIds.reserve(numKeypoints);

    // We go through all the 2D kps in new keyframe
    for (size_t i = 0; i < numKeypoints; i++)
    {
        // Get the related map point and check if it is ready to be triangulated
        std::shared_ptr<MapPoint> mapPoint = mapManager_->getMapPoint(keypoints.at(i).keypointId_);

        if (mapPoint == nullptr)
        {
            mapManager_->removeMapPointObs(keypoints.at(i).keypointId_, frame.keyframeId_);
            continue;
        }

        // If map point is already 3D continue (should not happen)
        if (mapPoint->is3d_)
        {
            continue;
        }

        // Get the set of keyframes sharing observation of this 2D map point
        std::set<int> coKeyframeIds = mapPoint->getObservedKeyframeIds();

        // Continue if new keyframe is the only one observing it
        if (coKeyframeIds.size() < 2)
        {
            continue;
        }

        int keyframeId = *coKeyframeIds.begin();

        if (frame.keyframeId_ == keyframeId)
        {
            continue;
        }

        // Get the 1st keyframe observation of the related map point
        keyframe = mapManager_->getKeyframe(keyframeId);

        if (keyframe == nullptr)
        {
            continue;
        }

        // Compute relative motion between new keyframe and selected keyframe (only if req.)
        if (relKeyframeId != keyframeId)
        {
            Sophus::SE3d Tciw = keyframe->getTcw();
            Tcicj = Tciw * Twcj;
            Tcjci = Tcicj.inverse();
            Rcicj = Tcicj.rotationMatrix();

            relKeyframeId = keyframeId;
        }

        Keypoint keyframeKeypoint = keyframe->getKeypointById(keypoints.at(i).keypointId_);

        if (keyframeKeypoint.keypointId_ != keypoints.at(i).keypointId_)
        {
            continue;
        }

        // Check rotation-compensated parallax
        cv::Point2f rotPx = frame.projCamToImage(Rcicj * keypoints.at(i).bv_);
        double parallax = cv::norm(keyframeKeypoint.unpx_ - rotPx);

        candidates++;

        // Compute 3D pos and check if its good or not
        lPoint = MultiViewGeometry::triangulate(Tcicj, keyframeKeypoint.bv_, keypoints.at(i).bv_);

        // Project into right cam (new keyframe)
        rPoint = Tcjci * lPoint;

        // Ensure that the 3D map point is in front of both camera
        if (lPoint.z() < 0.1 || rPoint.z() < 0.1)
        {
            if (parallax > 20.)
            {
                mapManager_->removeMapPointObs(keyframeKeypoint.keypointId_, frame.keyframeId_);
            }
            continue;
        }

        // Remove map point with high reprojection error
        lPxProj = keyframe->projCamToImage(lPoint);
        rPxProj = frame.projCamToImage(rPoint);
        lDist = cv::norm(lPxProj - keyframeKeypoint.unpx_);
        rDist = cv::norm(rPxProj - keypoints.at(i).unpx_);

        if (lDist > state_->mapMaxReprojectionError_ || rDist > state_->mapMaxReprojectionError_)
        {
            if (parallax > 20.)
            {
                mapManager_->removeMapPointObs(keyframeKeypoint.keypointId_, frame.keyframeId_);
            }
            continue;
        }

        // The 3D pos is good, update SLAM map point and related keyframe / Frame
        wpt = keyframe->projCamToWorld(lPoint);
        mapManager_->updateMapPoint(keypoints.at(i).keypointId_, wpt, 1. / lPoint.z());

        good++;
    }

    if (state_->debug_)
    {
        std::cout << "\n \t >>> Temporal Mapping : " << good << " 3D map points out of " << candidates << " kps !\n";
    }
}

bool Mapper::matchingToLocalMap(Frame &frame)
{
    // Maximum number of map points to track
    const size_t maxNumLocalMapPoints = state_->frameMaxNumKeypoints_ * 10;

    // If room for more keypoints, get local map of oldest co-keyframe and add it to set of map points to search for
    auto covMap = frame.getCovisibleKeyframeMap();

    if (frame.localMapPointIds_.size() < maxNumLocalMapPoints)
    {
        int keyframeId = covMap.begin()->first;
        auto keyframe = mapManager_->getKeyframe(keyframeId);
        while (keyframe == nullptr && keyframeId > 0 && !timedOperationHasTimedOut())
        {
            keyframeId--;
            keyframe = mapManager_->getKeyframe(keyframeId);
        }

        // Skip if no time
        if (timedOperationHasTimedOut())
        {
            return false;
        }

        if (keyframe != nullptr)
        {
            frame.localMapPointIds_.insert(keyframe->localMapPointIds_.begin(), keyframe->localMapPointIds_.end());
        }

        // If still far not enough, go for another round
        if (keyframe->keyframeId_ > 0 && frame.localMapPointIds_.size() < 0.5 * maxNumLocalMapPoints)
        {
            keyframe = mapManager_->getKeyframe(keyframe->keyframeId_);
            while (keyframe == nullptr && keyframeId > 0 && !timedOperationHasTimedOut())
            {
                keyframeId--;
                keyframe = mapManager_->getKeyframe(keyframeId);
            }

            // Skip if no time
            if (timedOperationHasTimedOut())
            {
                return false;
            }

            if (keyframe != nullptr)
            {
                frame.localMapPointIds_.insert(keyframe->localMapPointIds_.begin(), keyframe->localMapPointIds_.end());
            }
        }
    }

    // Skip if no time
    if (timedOperationHasTimedOut())
    {
        return false;
    }

    if (state_->debug_)
    {
        std::cout << "\n \t>>> matchToLocalMap() --> Number of local map points selected : " << frame.localMapPointIds_.size() << "\n";
    }

    // Track local map
    std::map<int, int> mapPrevIdNewId = matchToMap(frame, state_->mapMaxProjectionPxDistance_, state_->mapMaxDescriptorDistance_, frame.localMapPointIds_);

    size_t numMatches = mapPrevIdNewId.size();

    if (state_->debug_)
    {
        std::cout << "\n \t>>> matchToLocalMap() --> Match To Local Map found #" << numMatches << " matches \n";
    }

    // Return if no matches
    if (numMatches == 0)
    {
        return false;
    }

    // Merge in a thread to avoid waiting for BA to finish
    mergeMatches(frame, mapPrevIdNewId);

    return true;
}

void Mapper::mergeMatches(const Frame &frame, const std::map<int, int> &mapOfKeypointIdsAndMapPointIds)
{
    // Merge the matches
    for (const auto &ids: mapOfKeypointIdsAndMapPointIds)
    {
        int prevMapPointId = ids.first;
        int newMapPointId = ids.second;

        mapManager_->mergeMapPoints(prevMapPointId, newMapPointId);
    }

    if (state_->debug_)
    {
        std::cout << "\n >>> matchToLocalMap() / mergeMatches() --> Number of merges : " << mapOfKeypointIdsAndMapPointIds.size() << "\n";
    }
}

std::map<int, int> Mapper::matchToMap(const Frame &frame, const float maxProjectionError, const float distRatio, std::unordered_set<int> &localMapPointIds)
{
    std::map<int, int> mapPrevIdNewId;

    // Leave if local map is empty
    if (localMapPointIds.empty())
    {
        return mapPrevIdNewId;
    }

    // Compute max field of view
    const float fovV = 0.5 * frame.cameraCalibration_->imgHeight_ / frame.cameraCalibration_->fy_;
    const float fovH = 0.5 * frame.cameraCalibration_->imgWidth_ / frame.cameraCalibration_->fx_;

    float maxRadFov = 0.;
    if (fovH > fovV)
    {
        maxRadFov = std::atan(fovH);
    }
    else
    {
        maxRadFov = std::atan(fovV);
    }

    const float view_th = std::cos(maxRadFov);

    // Define max distance from projection
    float maxPxDist = maxProjectionError;
    if (frame.numKeypoints3d_ < 30)
    {
        maxPxDist *= 2.;
    }

    std::map<int, std::vector<std::pair<int, float>>> keypointIdsMapPointIdsDist;

    // Go through all map point from the local map
    for (const int mapPointId: localMapPointIds)
    {
        if (timedOperationHasTimedOut())
        {
            break;
        }

        if (frame.isObservingKeypoint(mapPointId))
        {
            continue;
        }

        auto mapPoint = mapManager_->getMapPoint(mapPointId);

        if (mapPoint == nullptr)
        {
            continue;
        }
        else if (!mapPoint->is3d_ || mapPoint->desc_.empty())
        {
            continue;
        }

        Eigen::Vector3d wpt = mapPoint->getPoint();

        //Project 3D map point into keyframe's image
        Eigen::Vector3d campt = frame.projWorldToCam(wpt);

        if (campt.z() < 0.1)
        {
            continue;
        }

        float view_angle = campt.z() / campt.norm();

        if (fabs(view_angle) < view_th)
        {
            continue;
        }

        cv::Point2f projPx = frame.projCamToImageDist(campt);

        if (!frame.isInImage(projPx))
        {
            continue;
        }

        // Get all the kps around the map point's projection
        auto nearKeyPoints = frame.getSurroundingKeypoints(projPx);

        // Find two best matches
        float minDist = mapPoint->desc_.cols * distRatio * 8.; // * 8 to get bits size
        int bestId = -1;
        int secId = -1;

        float bestDist = minDist;
        float secDist = minDist;

        std::vector<int> keypointIds;
        std::vector<float> pxDistances;
        cv::Mat descriptors;

        for (const auto &kp: nearKeyPoints)
        {
            if (kp.keypointId_ < 0)
            {
                continue;
            }

            float pxDist = cv::norm(projPx - kp.px_);

            if (pxDist > maxPxDist)
            {
                continue;
            }

            // Check that this kp and the map point are indeed candidates for matching
            // (by ensuring that they are never both observed in a given keyframe)
            auto kpMapPoint = mapManager_->getMapPoint(kp.keypointId_);

            if (kpMapPoint == nullptr)
            {
                mapManager_->removeMapPointObs(kp.keypointId_, frame.keyframeId_);
                continue;
            }

            if (kpMapPoint->desc_.empty())
            {
                continue;
            }

            bool isCandidate = true;
            auto mapPointKeyframes = mapPoint->getObservedKeyframeIds();

            for (const auto &keyframeId: kpMapPoint->getObservedKeyframeIds())
            {
                if (mapPointKeyframes.count(keyframeId))
                {
                    isCandidate = false;
                    break;
                }
            }

            if (!isCandidate)
            {
                continue;
            }

            float coProjectionPx = 0.;
            size_t numCoKeyPoints = 0;

            for (const auto &keyframeId: kpMapPoint->getObservedKeyframeIds())
            {
                auto coKeyframe = mapManager_->getKeyframe(keyframeId);
                if (coKeyframe != nullptr)
                {
                    auto cokp = coKeyframe->getKeypointById(kp.keypointId_);
                    if (cokp.keypointId_ == kp.keypointId_)
                    {
                        coProjectionPx += cv::norm(cokp.px_ - coKeyframe->projWorldToImageDist(wpt));
                        numCoKeyPoints++;
                    }
                    else
                    {
                        mapManager_->removeMapPointObs(kp.keypointId_, keyframeId);
                    }
                }
                else
                {
                    mapManager_->removeMapPointObs(kp.keypointId_, keyframeId);
                }
            }

            if (coProjectionPx / numCoKeyPoints > maxPxDist)
            {
                continue;
            }

            float dist = mapPoint->computeMinDescDist(*kpMapPoint);

            if (dist <= bestDist)
            {
                secDist = bestDist; // Will stay at minDist 1st time
                secId = bestId; // Will stay at -1 1st time

                bestDist = dist;
                bestId = kp.keypointId_;
            }
            else if (dist <= secDist)
            {
                secDist = dist;
                secId = kp.keypointId_;
            }
        }

        if (bestId != -1 && secId != -1)
        {
            if (0.9 * secDist < bestDist)
            {
                bestId = -1;
            }
        }

        if (bestId < 0)
        {
            continue;
        }

        std::pair<int, float> midDist(mapPointId, bestDist);
        if (!keypointIdsMapPointIdsDist.count(bestId))
        {
            std::vector<std::pair<int, float>> v(1, midDist);
            keypointIdsMapPointIdsDist.emplace(bestId, v);
        }
        else
        {
            keypointIdsMapPointIdsDist.at(bestId).push_back(midDist);
        }
    }

    for (const auto &keypointIdMapPointDist: keypointIdsMapPointIdsDist)
    {
        int keypointId = keypointIdMapPointDist.first;

        float bestDist = 1024;
        int bestMapPointId = -1;

        for (const auto &mapPointDist: keypointIdMapPointDist.second)
        {
            if (mapPointDist.second <= bestDist)
            {
                bestDist = mapPointDist.second;
                bestMapPointId = mapPointDist.first;
            }
        }

        if (bestMapPointId >= 0)
        {
            mapPrevIdNewId.emplace(keypointId, bestMapPointId);
        }
    }

    return mapPrevIdNewId;
}

void Mapper::timedOperationStart()
{
    timedOperationStartTime_ = std::chrono::high_resolution_clock::now();
}

bool Mapper::timedOperationHasTimedOut()
{
    auto now = std::chrono::high_resolution_clock::now();
    auto dif = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - timedOperationStartTime_).count();

    return (dif > 8);
}
