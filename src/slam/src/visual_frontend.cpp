#include "visual_frontend.hpp"
#include "multi_view_geometry.hpp"
#include <opencv2/video/tracking.hpp>

VisualFrontend::VisualFrontend(std::shared_ptr<State> state,
                               std::shared_ptr<Frame> frame,
                               std::shared_ptr<MapManager> mapManager,
                               std::shared_ptr<Mapper> mapper,
                               std::shared_ptr<FeatureTracker> featureTracker) :
        state_(state),
        currFrame_(frame),
        mapManager_(mapManager),
        mapper_(mapper),
        featureTracker_(featureTracker)
{
    cv::Size gridSize(state_->imgWidth_ / state_->claheTileSize_, state_->imgHeight_ / state_->claheTileSize_);

    clahe_ = cv::createCLAHE(state_->claheContrastLimit_, gridSize);
}

void VisualFrontend::track(cv::Mat &image, double timestamp)
{
    bool isKeyFrameRequired = process(image, timestamp);

    if (isKeyFrameRequired)
    {
        mapManager_->createKeyframe(currImage_, image);

        if (state_->trackKeyframeToFrame_)
        {
            cv::buildOpticalFlowPyramid(currImage_, keyframePyramid_, state_->kltWinSize_, state_->kltPyramidLevels_);
        }

        if (!state_->slamResetRequested_ && state_->slamReadyForInit_)
        {
            Keyframe kf(currFrame_->keyframeId_, image);
            mapper_->processNewKeyframe(kf);
        }
    }
}

bool VisualFrontend::process(cv::Mat &image, double timestamp)
{
    preprocessImage(image);

    // Create keyframe if this is the first frame
    if (currFrame_->id_ == 0)
    {
        return true;
    }

    // Apply motion model to predict current frame pose
    Sophus::SE3d Twc = currFrame_->getTwc();
    motionModel_.applyMotionModel(Twc, timestamp);
    currFrame_->setTwc(Twc);

    // Track the new image
    if (state_->trackKeyframeToFrame_)
    {
        kltTrackingFromKeyframe();
    }
    else
    {
        kltTrackingFromMotionPrior();
    }

    //epipolar2d2dOutlierFiltering();

    if (!state_->slamReadyForInit_)
    {
        if (currFrame_->numKeypoints2d_ < 50)
        {
            state_->slamResetRequested_ = true;
            return false;
        }
        else if (checkReadyForInit())
        {
            state_->slamReadyForInit_ = true;
            return true;
        }
        else
        {
            std::cout << "- [Visual-Front-End]: Not ready for initialization\n";
            return false;
        }
    }
    else
    {
        bool success = computePose();

        if (!success)
        {
            std::cout << "- [Visual-Front-End]: Failed to compute pose num times: " << poseFailedCounter_ << std::endl;

            poseFailedCounter_++;

            if (poseFailedCounter_ > 3)
            {
                state_->slamResetRequested_ = true;
                return false;
            }
        }

        motionModel_.updateMotionModel(currFrame_->Twc_, timestamp);

        bool keyFrameRequired = checkNewKeyframeRequired();

        return keyFrameRequired;
    }
}

void VisualFrontend::kltTrackingFromMotionPrior()
{
    // Get current kps and init priors for tracking
    std::vector<int> v3dkpids;
    std::vector<int> vkpids;
    std::vector<cv::Point2f> v3dkps;
    std::vector<cv::Point2f> v3dpriors;
    std::vector<cv::Point2f> vkps;
    std::vector<cv::Point2f> vpriors;
    std::vector<bool> vkpis3d;

    // First track 3d kps on only 2 levels
    v3dkpids.reserve(currFrame_->numKeypoints3d_);
    v3dkps.reserve(currFrame_->numKeypoints3d_);
    v3dpriors.reserve(currFrame_->numKeypoints3d_);

    // Then track 2d kps on full pyramid levels
    vkpids.reserve(currFrame_->numKeypoints_);
    vkps.reserve(currFrame_->numKeypoints_);
    vpriors.reserve(currFrame_->numKeypoints_);
    vkpis3d.reserve(currFrame_->numKeypoints_);

    for (const auto &it: currFrame_->mapKeypoints_)
    {
        auto &keypoint = it.second;

        // Init prior px pos. from motion model
        if (state_->kltUsePrior_)
        {
            if (keypoint.is3d_)
            {
                cv::Point2f projpx = currFrame_->projWorldToImageDist(mapManager_->mapMapPoints_.at(keypoint.keypointId_)->getPoint());

                // Add prior if projected into image
                if (currFrame_->isInImage(projpx))
                {
                    v3dkps.push_back(keypoint.px_);
                    v3dpriors.push_back(projpx);
                    v3dkpids.push_back(keypoint.keypointId_);
                    vkpis3d.push_back(true);
                    continue;
                }
            }
        }

        // For other kps init prior with prev px pos.
        vkpids.push_back(keypoint.keypointId_);
        vkps.push_back(keypoint.px_);
        vpriors.push_back(keypoint.px_);
    }

    // 1st track 3d kps if using prior
    if (state_->kltUsePrior_ && !v3dpriors.empty())
    {
        // Good / bad kps vector
        std::vector<bool> keypointStatus;

        auto vprior = v3dpriors;

        featureTracker_->fbKltTracking(
                prevPyramid_,
                currPyramid_,
                state_->kltWinSizeWH_,
                1, // number of pyramid levels
                state_->kltError_,
                state_->kltMaxFbDistance_,
                v3dkps,
                v3dpriors,
                keypointStatus);

        size_t numGood = 0;
        size_t numKeypoints = v3dkps.size();

        for (size_t i = 0; i < numKeypoints; i++)
        {
            if (keypointStatus.at(i))
            {
                currFrame_->updateKeypoint(v3dkpids.at(i), v3dpriors.at(i));
                numGood++;
            }
            else
            {
                // If tracking failed, try on full pyramid size
                vkpids.push_back(v3dkpids.at(i));
                vkps.push_back(v3dkps.at(i));
                vpriors.push_back(v3dpriors.at(i));
            }
        }

        if (state_->debug_)
        {
            std::cout << "- [Visual-Front-End]: KLT Tracking w. priors : " << numGood << " out of " << numKeypoints << " kps tracked!\n";
        }

        if (numGood < 0.33 * numKeypoints)
        {
            // Motion model might be quite wrong, P3P is recommended next and not using any prior
            p3pReq_ = true;
            vpriors = vkps;
        }
    }

    // 2nd track other keypoints if any
    if (!vkps.empty())
    {
        // Good / bad kps vector
        std::vector<bool> keypointStatus;

        featureTracker_->fbKltTracking(
                prevPyramid_,
                currPyramid_,
                state_->kltWinSizeWH_,
                state_->kltPyramidLevels_,
                state_->kltError_,
                state_->kltMaxFbDistance_,
                vkps,
                vpriors,
                keypointStatus);

        size_t numGood = 0;
        size_t numKeypoints = vkps.size();

        for (size_t i = 0; i < numKeypoints; i++)
        {
            if (keypointStatus.at(i))
            {
                currFrame_->updateKeypoint(vkpids.at(i), vpriors.at(i));
                numGood++;
            }
            else
            {
                mapManager_->removeObsFromCurrFrameById(vkpids.at(i));
            }
        }

        if (state_->debug_)
        {
            std::cout << "- [Visual-Front-End]: KLT Tracking no prior : " << numGood << " out of " << numKeypoints << " kps tracked!\n";
        }
    }
}

void VisualFrontend::kltTrackingFromKeyframe()
{
    // Get current kps and init priors for tracking
    std::vector<int> keyPoint3dIds;
    std::vector<int> keyPointIds;
    std::vector<cv::Point2f> keyPoints3d;
    std::vector<cv::Point2f> priors3d;
    std::vector<cv::Point2f> keyPoints;
    std::vector<cv::Point2f> priors;
    std::vector<bool> keypointIs3d;

    // First track 3d kps on only 2 levels
    keyPoint3dIds.reserve(currFrame_->numKeypoints3d_);
    keyPoints3d.reserve(currFrame_->numKeypoints3d_);
    priors3d.reserve(currFrame_->numKeypoints3d_);

    // Then track 2d kps on full pyramid levels
    keyPointIds.reserve(currFrame_->numKeypoints_);
    keyPoints.reserve(currFrame_->numKeypoints_);
    priors.reserve(currFrame_->numKeypoints_);
    keypointIs3d.reserve(currFrame_->numKeypoints_);

    // Get prev keyframe
    auto pkf = mapManager_->mapKeyframes_.at(currFrame_->keyframeId_);

    if (pkf == nullptr)
    {
        return;
    }

    std::vector<int> badIds;
    badIds.reserve(currFrame_->numKeypoints_ * 0.2);

    for (const auto &it: currFrame_->mapKeypoints_)
    {
        auto &keypoint = it.second;
        auto kfkpit = pkf->mapKeypoints_.find(keypoint.keypointId_);

        if (kfkpit == pkf->mapKeypoints_.end())
        {
            badIds.push_back(keypoint.keypointId_);
            continue;
        }

        // Init prior px pos. from motion model
        if (state_->kltUsePrior_)
        {
            if (keypoint.is3d_)
            {
                cv::Point2f projpx = currFrame_->projWorldToImageDist(mapManager_->mapMapPoints_.at(keypoint.keypointId_)->getPoint());

                // Add prior if projected into image
                if (currFrame_->isInImage(projpx))
                {
                    keyPoints3d.push_back(kfkpit->second.px_);
                    priors3d.push_back(projpx);
                    keyPoint3dIds.push_back(keypoint.keypointId_);
                    keypointIs3d.push_back(true);
                    continue;
                }
            }
        }

        // For other kps init prior with prev px pos.
        keyPointIds.push_back(keypoint.keypointId_);
        keyPoints.push_back(kfkpit->second.px_);
        priors.push_back(keypoint.px_);
    }

    for (const auto &id: badIds)
    {
        mapManager_->removeObsFromCurrFrameById(id);
    }

    // 1st track 3d key points if using priors
    if (state_->kltUsePrior_ && !priors3d.empty())
    {
        // Good / bad kps vector
        std::vector<bool> keyPointStatus;

        auto vprior = priors3d;

        featureTracker_->fbKltTracking(
                keyframePyramid_,
                currPyramid_,
                state_->kltWinSizeWH_,
                1,
                state_->kltError_,
                state_->kltMaxFbDistance_,
                keyPoints3d,
                priors3d,
                keyPointStatus);

        size_t numGood = 0;
        size_t numKeypoints = keyPoints3d.size();

        for (size_t i = 0; i < numKeypoints; i++)
        {
            if (keyPointStatus.at(i))
            {
                currFrame_->updateKeypoint(keyPoint3dIds.at(i), priors3d.at(i));
                numGood++;
            }
            else
            {
                // If tracking failed, try full pyramid size
                keyPointIds.push_back(keyPoint3dIds.at(i));
                keyPoints.push_back(keyPoints3d.at(i));
                priors.push_back(currFrame_->mapKeypoints_.at(keyPoint3dIds.at(i)).px_);
            }
        }

        if (state_->debug_)
        {
            std::cout << "- [Visual-Front-End]: KLT Tracking w. priors : " << numGood << " out of " << numKeypoints << " kps tracked!\n";
        }

        if (numGood < 0.33 * numKeypoints)
        {
            // Motion model might be quite wrong, P3P is recommended next and not using any prior
            p3pReq_ = true;
            priors = keyPoints;
        }
    }

    // 2nd track other key points if any
    if (!keyPoints.empty())
    {
        // Good / bad kps vector
        std::vector<bool> keypointStatus;

        featureTracker_->fbKltTracking(
                keyframePyramid_,
                currPyramid_,
                state_->kltWinSizeWH_,
                state_->kltPyramidLevels_,
                state_->kltError_,
                state_->kltMaxFbDistance_,
                keyPoints,
                priors,
                keypointStatus);

        size_t numGood = 0;
        size_t numKeypoints = keyPoints.size();

        for (size_t i = 0; i < numKeypoints; i++)
        {
            if (keypointStatus.at(i))
            {
                currFrame_->updateKeypoint(keyPointIds.at(i), priors.at(i));
                numGood++;
            }
            else
            {
                mapManager_->removeObsFromCurrFrameById(keyPointIds.at(i));
            }
        }

        if (state_->debug_)
        {
            std::cout << "- [Visual-Front-End]: KLT Tracking no prior : " << numGood << " out of " << numKeypoints << " kps tracked!\n";
        }
    }
}

void VisualFrontend::epipolar2d2dOutlierFiltering()
{
    auto keyframe = mapManager_->mapKeyframes_.at(currFrame_->keyframeId_);

    if (keyframe == nullptr)
    {
        std::cerr << "- [Visual-Front-End]: ERROR! Previous Kf does not exist yet (epipolar2d2d()).\n";
        exit(-1);
    }

    // Get current frame number kps
    size_t numKeypoints = currFrame_->numKeypoints_;

    if (numKeypoints < 8)
    {
        std::cout << "- [Visual-Front-End]: Not enough kps to compute Essential Matrix" << std::endl;
        return;
    }

    // Setup Essential Matrix computation for OpenGV-based filtering
    std::vector<int> keypointIds;
    std::vector<int> outliersIndices;
    keypointIds.reserve(numKeypoints);
    outliersIndices.reserve(numKeypoints);

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > vkfbvs;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > vcurbvs;
    vkfbvs.reserve(numKeypoints);
    vcurbvs.reserve(numKeypoints);

    size_t numParallax = 0;
    float avgParallax = 0.;

    // Compute rotation compensated parallax
    Eigen::Matrix3d Rkfcur = keyframe->getRcw() * currFrame_->getRwc();

    // Init bearing vectors and check parallax
    for (const auto &it: currFrame_->mapKeypoints_)
    {
        auto &keypoint = it.second;

        // Get the prev keyframe related kp if it exists
        auto keyframeKeypoint = keyframe->getKeypointById(keypoint.keypointId_);

        if (keyframeKeypoint.keypointId_ != keypoint.keypointId_)
        {
            continue;
        }

        // Store the bvs and their ids
        vkfbvs.push_back(keyframeKeypoint.bv_);
        vcurbvs.push_back(keypoint.bv_);
        keypointIds.push_back(keypoint.keypointId_);

        cv::Point2f rotPx = keyframe->projCamToImage(Rkfcur * keypoint.bv_);

        // Compute parallax
        avgParallax += cv::norm(rotPx - keyframeKeypoint.unpx_);
        numParallax++;
    }

    // Average parallax
    avgParallax /= numParallax;

    if (avgParallax < 2.0 * state_->multiViewRansacError_)
    {
        if (state_->debug_)
        {
            std::cout << "- [Visual-Front-End]: Not enough parallax (" << avgParallax << " px) to compute 5-pt Essential Matrix\n";
        }
        return;
    }

    bool doOptimize = false;

    // use the resulting motion if tracking is poor
    if (mapManager_->numKeyframes_ > 2 && currFrame_->numKeypoints3d_ < 30)
    {
        doOptimize = true;
    }

    Eigen::Matrix3d Rkfc;
    Eigen::Vector3d tkfc;

    if (state_->debug_)
    {
        std::cout << "\n \t>>> 5-pt EssentialMatrix Ransac :";
        std::cout << "\n \t>>> nb pts : " << numKeypoints;
        std::cout << " / avg. parallax : " << avgParallax;
        std::cout << " / ransac_iterations : " << state_->multiViewRansacNumIterations_;
        std::cout << " / ransac_error : " << state_->multiViewRansacError_;
        std::cout << "\n\n";
    }

    bool success = MultiViewGeometry::compute5ptEssentialMatrix(
            vkfbvs,
            vcurbvs,
            state_->multiViewRansacNumIterations_,
            state_->multiViewRansacError_,
            doOptimize,
            state_->multiViewRandomEnabled_,
            currFrame_->cameraCalibration_->fx_,
            currFrame_->cameraCalibration_->fy_,
            Rkfc,
            tkfc,
            outliersIndices);

    if (state_->debug_)
    {
        std::cout << "- [Visual-Front-End]: Epipolar nb outliers : " << outliersIndices.size();
    }

    if (!success)
    {
        std::cout << "- [Visual-Front-End]: No pose could be computed from 5-pt EssentialMatrix" << std::endl;
        return;
    }

    if (outliersIndices.size() > 0.5 * vkfbvs.size())
    {
        std::cout << "- [Visual-Front-End]: Too many outliers, skipping as might be degenerate case" << std::endl;
        return;
    }

    // Remove outliers
    for (const auto &idx: outliersIndices)
    {
        mapManager_->removeObsFromCurrFrameById(keypointIds.at(idx));
    }

    // In case we wanted to use the resulting motion (can help when tracking is poor)
    if (doOptimize && mapManager_->numKeyframes_ > 2)
    {
        // Get motion model translation scale from last keyframe
        Sophus::SE3d Tkfw = keyframe->getTcw();
        Sophus::SE3d Tkfcur = Tkfw * currFrame_->getTwc();

        double scale = Tkfcur.translation().norm();
        tkfc.normalize();

        // Update current pose with Essential Mat. relative motion and current trans. scale
        Sophus::SE3d Tkfc(Rkfc, scale * tkfc);

        currFrame_->setTwc(keyframe->getTwc() * Tkfc);
    }
}

bool VisualFrontend::computePose()
{
    size_t num3dKeypoints = currFrame_->numKeypoints3d_;

    if (num3dKeypoints < 4)
    {
        if (state_->debug_)
        {
            std::cout << "- [Visual-Front-End]: Not enough kps to compute P3P/PnP\n";
        }

        return false;
    }

    // setup P3P-Ransac computation for OpenGV-based Pose estimation + motion-only BA with Ceres
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > vbvs;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > vwpts;
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > vkps;
    std::vector<int> vkpids;
    std::vector<int> outliersIndices;

    vbvs.reserve(num3dKeypoints);
    vwpts.reserve(num3dKeypoints);
    vkpids.reserve(num3dKeypoints);
    outliersIndices.reserve(num3dKeypoints);
    vkps.reserve(num3dKeypoints);

    bool doP3P = p3pReq_ || state_->p3pEnabled_;

    // store every 3d bvs, map points and their related ids
    for (const auto &it: currFrame_->mapKeypoints_)
    {
        if (!it.second.is3d_)
        {
            continue;
        }

        auto &kp = it.second;
        auto plm = mapManager_->mapMapPoints_.at(kp.keypointId_);

        if (plm == nullptr)
        {
            continue;
        }

        if (doP3P)
        {
            vbvs.push_back(kp.bv_);
        }

        vkps.push_back(Eigen::Vector2d(kp.unpx_.x, kp.unpx_.y));
        vwpts.push_back(plm->getPoint());
        vkpids.push_back(kp.keypointId_);
    }

    Sophus::SE3d Twc = currFrame_->getTwc();
    bool doOptimize = false;
    bool success = false;

    if (doP3P)
    {
        if (state_->debug_)
        {
            std::cout << "\n \t>>>P3P Ransac : ";
            std::cout << "\n \t>>> nb 3d pts : " << num3dKeypoints;
            std::cout << " / ransac_iterations : " << state_->multiViewRansacNumIterations_;
            std::cout << " / ransac_error : " << state_->multiViewRansacError_;
            std::cout << "\n\n";
        }

        success = MultiViewGeometry::p3pRansac(
                vbvs,
                vwpts,
                state_->multiViewRansacNumIterations_,
                state_->multiViewRansacError_,
                doOptimize,
                state_->multiViewRandomEnabled_,
                currFrame_->cameraCalibration_->fx_,
                currFrame_->cameraCalibration_->fy_,
                Twc,
                outliersIndices,
                true //use meds, only effective with OpenGV
        );

        if (state_->debug_)
        {
            std::cout << "- [Visual-Front-End]: P3P-LMeds nb outliers : " << outliersIndices.size();
        }

        // Check that pose estimation was good enough
        size_t numInliers = vwpts.size() - outliersIndices.size();

        if (!success || numInliers < 5 || Twc.translation().array().isInf().any() || Twc.translation().array().isNaN().any())
        {
            if (state_->debug_)
            {
                std::cout << "- [Visual-Front-End]: Not enough inliers for reliable pose est. Resetting keyframe state\n";
            }

            resetFrame();

            return false;
        }

        // pose seems to be OK!

        // update frame pose
        currFrame_->setTwc(Twc);

        // Remove outliers before PnP refinement (a bit dirty)
        int k = 0;
        for (const auto &index: outliersIndices)
        {
            mapManager_->removeObsFromCurrFrameById(vkpids.at(index - k));
            vkps.erase(vkps.begin() + index - k);
            vwpts.erase(vwpts.begin() + index - k);
            vkpids.erase(vkpids.begin() + index - k);
            k++;
        }

        // Clear before robust PnP refinement using Ceres
        outliersIndices.clear();
    }

    // Ceres-based PnP (motion-only BA)
    bool useRobust = true;
    size_t maxIterations = 5;

    success = MultiViewGeometry::ceresPnP(
            vkps,
            vwpts,
            Twc,
            maxIterations,
            state_->baRobustThreshold_,
            useRobust,
            state_->applyL2AfterRobust_,
            currFrame_->cameraCalibration_->fx_,
            currFrame_->cameraCalibration_->fy_,
            currFrame_->cameraCalibration_->cx_,
            currFrame_->cameraCalibration_->cy_,
            outliersIndices);

    // Check that pose estimation was good enough
    size_t numInliers = vwpts.size() - outliersIndices.size();

    if (state_->debug_)
    {
        std::cout << "- [Visual-Front-End]: Ceres PnP outliers : " << outliersIndices.size() << ", inliers: " << numInliers << "\n";
    }

    if (!success || numInliers < 5 || outliersIndices.size() > 0.5 * vwpts.size() || Twc.translation().array().isInf().any() || Twc.translation().array().isNaN().any())
    {
        if (!doP3P)
        {
            // Weird results, skipping here and applying p3p next
            p3pReq_ = true;
        }

        if (state_->debug_)
        {
            std::cout << "- [Visual-Front-End]: Not enough inliers for reliable pose est. Resetting keyframe state\n";
        }

        resetFrame();

        return false;
    }

    // Pose seems to be OK

    // Update frame pose
    currFrame_->setTwc(Twc);

    // Set p3p req to false as it is triggered either because of bad PnP or by bad klt tracking
    p3pReq_ = false;

    for (const auto &idx: outliersIndices)
    {
        mapManager_->removeObsFromCurrFrameById(vkpids.at(idx));
    }

    return true;
}

bool VisualFrontend::checkReadyForInit()
{
    double avgComputedRotParallax = computeParallax(currFrame_->keyframeId_, false, true);

    if (avgComputedRotParallax <= state_->minAvgRotationParallax_)
    {
        return false;
    }

    auto prevKeyframe = mapManager_->mapKeyframes_.at(currFrame_->keyframeId_);

    if (prevKeyframe == nullptr)
    {
        return false;
    }

    size_t numKeypoints = currFrame_->numKeypoints_;

    if (numKeypoints < 8)
    {
        std::cout << "- [Visual-Front-End]: Can't compute 5-pt Essential matrix. Not enough keypoints.\n";
        return false;
    }

    // Setup Essential Matrix computation for OpenGV-based filtering
    std::vector<int> keypointIds;
    std::vector<int> outliersIndices;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > vkfbvs;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > vcurbvs;

    keypointIds.reserve(numKeypoints);
    outliersIndices.reserve(numKeypoints);
    vkfbvs.reserve(numKeypoints);
    vcurbvs.reserve(numKeypoints);

    Eigen::Matrix3d RCurrKeyframe = prevKeyframe->getTcw().rotationMatrix() * currFrame_->getTwc().rotationMatrix();
    int numParallax = 0;
    float avgRotParallax = 0.;

    // Get bvs and compute the rotation compensated parallax for all current keypoints
    for (const auto &it: currFrame_->mapKeypoints_)
    {
        auto &keypoint = it.second;

        // get the prev keyframe related kp if it exists
        auto keyframeKeypoint = prevKeyframe->getKeypointById(keypoint.keypointId_);

        if (keyframeKeypoint.keypointId_ != keypoint.keypointId_)
        {
            continue;
        }

        // Store the bvs and their ids
        vkfbvs.push_back(keyframeKeypoint.bv_);
        vcurbvs.push_back(keypoint.bv_);
        keypointIds.push_back(keypoint.keypointId_);

        // compute rotation compensated parallax
        Eigen::Vector3d rotBv = RCurrKeyframe * keypoint.bv_;
        Eigen::Vector3d unpx = currFrame_->cameraCalibration_->K_ * rotBv;
        cv::Point2f rotpx(unpx.x() / unpx.z(), unpx.y() / unpx.z());

        avgRotParallax += cv::norm(rotpx - keyframeKeypoint.unpx_);
        numParallax++;
    }

    if (numParallax < 8)
    {
        std::cout << "- [Visual-Front-End]: Can't compute 5-pt Essential matrix. Not enough prev keyframe keypoints.\n";
        return false;
    }

    // average parallax
    avgRotParallax /= float(numParallax);

    if (avgRotParallax < state_->minAvgRotationParallax_)
    {
        std::cout << "- [Visual-Front-End]: Can't compute 5-pt Essential matrix. Not enough parallax (\" << avgRotParallax << \" px)\n";
        return false;
    }

    Eigen::Matrix3d Rkfc;
    Eigen::Vector3d tkfc;
    Rkfc.setIdentity();
    tkfc.setZero();

    bool success = MultiViewGeometry::compute5ptEssentialMatrix(
            vkfbvs,
            vcurbvs,
            state_->multiViewRansacNumIterations_,
            state_->multiViewRansacError_,
            true, // optimize
            state_->multiViewRandomEnabled_,
            currFrame_->cameraCalibration_->fx_,
            currFrame_->cameraCalibration_->fy_,
            Rkfc,
            tkfc,
            outliersIndices);

    if (!success)
    {
        std::cout << "- [Visual-Front-End]: 5-pt Essential Matrix failed.\n";
        return false;
    }

    // Remove outliers from current frame
    for (const auto &idx: outliersIndices)
    {
        mapManager_->removeObsFromCurrFrameById(keypointIds.at(idx));
    }

    // Arbitrary scale. Exploring good default values
    tkfc.normalize();
    tkfc = tkfc.eval() * 0.25;
    //tkfc = tkfc.eval() * 0.1;
    //tkfc.z() = 0.0;

    currFrame_->setTwc(Rkfc, tkfc);

    return true;
}

bool VisualFrontend::checkNewKeyframeRequired()
{
    // get prev keyframe
    auto keyframeIt = mapManager_->mapKeyframes_.find(currFrame_->keyframeId_);

    if (keyframeIt == mapManager_->mapKeyframes_.end())
    {
        return false; // should not happen
    }

    auto keyframe = keyframeIt->second;

    // Compute median parallax unrot : false / median : true
    double medianRotParallax = computeParallax(keyframe->keyframeId_, true, true);

    int idDiff = currFrame_->id_ - keyframe->id_;

    if (idDiff >= 5 && currFrame_->numOccupiedCells_ < 0.33 * state_->frameMaxNumKeypoints_)
    {
        return true;
    }

    if (idDiff >= 2 && currFrame_->numKeypoints3d_ < 20)
    {
        return true;
    }

    if (idDiff < 2 && currFrame_->numKeypoints3d_ > 0.5 * state_->frameMaxNumKeypoints_)
    {
        return false;
    }

    bool cx = medianRotParallax >= state_->minAvgRotationParallax_ / 2.;
    bool c0 = medianRotParallax >= state_->minAvgRotationParallax_;
    bool c1 = currFrame_->numKeypoints3d_ < 0.75 * keyframe->numKeypoints3d_;
    bool c2 = currFrame_->numOccupiedCells_ < 0.5 * state_->frameMaxNumKeypoints_ && currFrame_->numKeypoints3d_ < 0.85 * keyframe->numKeypoints3d_;

    bool keyFrameRequired = (c0 || c1 || c2) && cx;

    return keyFrameRequired;
}

float VisualFrontend::computeParallax(const int keyframeId, bool doUnRotate, bool doMedian)
{
    // Get prev keyframe
    auto keyframeIt = mapManager_->mapKeyframes_.find(keyframeId);

    if (keyframeIt == mapManager_->mapKeyframes_.end())
    {
        return 0.;
    }

    // Compute relative rotation between current frame and previous keyframe if required
    Eigen::Matrix3d Rkfcur(Eigen::Matrix3d::Identity());

    if (doUnRotate)
    {
        Eigen::Matrix3d Rkfw = keyframeIt->second->getRcw();
        Eigen::Matrix3d Rwcur = currFrame_->getRwc();
        Rkfcur = Rkfw * Rwcur;
    }

    // Compute parallax
    float avgParallax = 0.;
    int numParallax = 0;

    std::set<float> parallaxSet;

    // Compute parallax for all keypoints seen in prev keyframe
    for (const auto &it: currFrame_->mapKeypoints_)
    {
        auto &kp = it.second;

        // get prev keyframe keypoint if it exists
        auto keypoint = keyframeIt->second->getKeypointById(kp.keypointId_);

        if (keypoint.keypointId_ != kp.keypointId_)
        {
            continue;
        }

        // Compute parallax with unpx position
        cv::Point2f unpx = kp.unpx_;

        // Rotate bv into keyframe cam frame and back project into image
        if (doUnRotate)
        {
            unpx = keyframeIt->second->projCamToImage(Rkfcur * kp.bv_);
        }

        // Compute rotation-compensated parallax
        float parallax = cv::norm(unpx - keypoint.unpx_);
        avgParallax += parallax;
        numParallax++;

        if (doMedian)
        {
            parallaxSet.insert(parallax);
        }
    }

    if (numParallax == 0)
    {
        return 0.;
    }

    avgParallax /= float(numParallax);

    if (doMedian)
    {
        auto it = parallaxSet.begin();
        std::advance(it, parallaxSet.size() / 2);
        avgParallax = *it;
    }

    return avgParallax;
}

void VisualFrontend::preprocessImage(cv::Mat &image)
{
    // update prev image
    if (!state_->trackKeyframeToFrame_)
    {
        cv::swap(currImage_, prevImage_);
    }

    // update curr image
    if (state_->claheEnabled_)
    {
        clahe_->apply(image, currImage_);
    }
    else
    {
        currImage_ = image;
    }

    // pre-building the pyramid used for KLT speed-up
    if (state_->kltEnabled_)
    {
        // If tracking from prev image, swap the pyramid
        if (!currPyramid_.empty() && !state_->trackKeyframeToFrame_)
        {
            prevPyramid_.swap(currPyramid_);
        }

        cv::buildOpticalFlowPyramid(currImage_, currPyramid_, state_->kltWinSize_, state_->kltPyramidLevels_);
    }
}

void VisualFrontend::resetFrame()
{
    auto mapKeypoints = currFrame_->mapKeypoints_;

    for (const auto &keypoint: mapKeypoints)
    {
        mapManager_->removeObsFromCurrFrameById(keypoint.first);
    }

    currFrame_->mapKeypoints_.clear();
    currFrame_->gridKeypointsIds_.clear();
    currFrame_->gridKeypointsIds_.resize(currFrame_->gridCells_);
    currFrame_->numKeypoints_ = 0;
    currFrame_->numKeypoints2d_ = 0;
    currFrame_->numKeypoints3d_ = 0;
    currFrame_->numOccupiedCells_ = 0;
}

void VisualFrontend::reset()
{
    currImage_.release();
    prevImage_.release();

    currPyramid_.clear();
    prevPyramid_.clear();
    keyframePyramid_.clear();

    poseFailedCounter_ = 0;
}
