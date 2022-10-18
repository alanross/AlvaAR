#include "visual_frontend.hpp"
#include "multi_view_geometry.hpp"
#include <opencv2/video/tracking.hpp>

VisualFrontend::VisualFrontend(std::shared_ptr<State> state, std::shared_ptr<Frame> frame, std::shared_ptr<MapManager> mapManager, std::shared_ptr<FeatureTracker> featureTracker)
        : state_(state), currFrame_(frame), mapManager_(mapManager), featureTracker_(featureTracker)
{
    int tileSize = 50;
    cv::Size clahe_tiles(state_->imgWidth_ / tileSize, state_->imgHeight_ / tileSize);

    clahe_ = cv::createCLAHE(state_->claheContrastLimit_, clahe_tiles);
}

bool VisualFrontend::visualTracking(cv::Mat &image, double timestamp)
{
    bool keyFrameRequired = track(image, timestamp);

    if (keyFrameRequired)
    {
        mapManager_->createKeyframe(currImage_, image);

        if (state_->trackKeyframeToFrame_)
        {
            cv::buildOpticalFlowPyramid(currImage_, keyframePyramid_, state_->kltWinSize_, state_->kltPyramidLevels_);
        }
    }

    return keyFrameRequired;
}

bool VisualFrontend::track(cv::Mat &image, double timestamp)
{
    // Preprocess the new image
    preprocessImage(image);

    // Create keyframe if 1st frame processed
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
        kltTracking();
    }

    // was optional at some point
    epipolar2d2dFiltering();

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

    bool foundPose = computePose();

    if (!foundPose)
    {
        std::cout << "- [Visual-Front-End]: Failed to compute pose\n";
    }

    motionModel_.updateMotionModel(currFrame_->Twc_, timestamp);

    bool keyFrameRequired = checkNewKeyframeRequired();

    return keyFrameRequired;
}

void VisualFrontend::kltTracking()
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
        auto &kp = it.second;

        // Init prior px pos. from motion model
        if (state_->kltUsePrior_)
        {
            if (kp.is3d_)
            {
                cv::Point2f projpx = currFrame_->projWorldToImageDist(mapManager_->mapMapPoints_.at(kp.keypointId_)->getPoint());

                // Add prior if projected into image
                if (currFrame_->isInImage(projpx))
                {
                    v3dkps.push_back(kp.px_);
                    v3dpriors.push_back(projpx);
                    v3dkpids.push_back(kp.keypointId_);

                    vkpis3d.push_back(true);
                    continue;
                }
            }
        }

        // For other kps init prior with prev px pos.
        vkpids.push_back(kp.keypointId_);
        vkps.push_back(kp.px_);
        vpriors.push_back(kp.px_);
    }

    // 1st track 3d kps if using prior
    if (state_->kltUsePrior_ && !v3dpriors.empty())
    {
        int nbpyrlvl = 1;

        // Good / bad kps vector
        std::vector<bool> vkpstatus;

        auto vprior = v3dpriors;

        featureTracker_->fbKltTracking(
                prevPyramid_,
                currPyramid_,
                state_->kltWinSizeWH_,
                nbpyrlvl,
                state_->kltError_,
                state_->kltMaxFbDistance_,
                v3dkps,
                v3dpriors,
                vkpstatus);

        size_t nbgood = 0;
        size_t nbkps = v3dkps.size();

        for (size_t i = 0; i < nbkps; i++)
        {
            if (vkpstatus.at(i))
            {
                currFrame_->updateKeypoint(v3dkpids.at(i), v3dpriors.at(i));
                nbgood++;
            }
            else
            {
                // If tracking failed, gonna try on full pyramid size
                vkpids.push_back(v3dkpids.at(i));
                vkps.push_back(v3dkps.at(i));
                vpriors.push_back(v3dpriors.at(i));
            }
        }

        if (state_->debug_)
        {
            std::cout << "- [Visual-Front-End]: KLT Tracking w. priors : " << nbgood << " out of " << nbkps << " kps tracked!\n";
        }

        if (nbgood < 0.33 * nbkps)
        {
            // Motion model might be quite wrong, P3P is recommended next and not using any prior
            p3pReq_ = true;
            vpriors = vkps;
        }
    }

    // 2nd track other kps if any
    if (!vkps.empty())
    {
        // Good / bad kps vector
        std::vector<bool> vkpstatus;

        featureTracker_->fbKltTracking(
                prevPyramid_,
                currPyramid_,
                state_->kltWinSizeWH_,
                state_->kltPyramidLevels_,
                state_->kltError_,
                state_->kltMaxFbDistance_,
                vkps,
                vpriors,
                vkpstatus);

        size_t nbgood = 0;
        size_t nbkps = vkps.size();

        for (size_t i = 0; i < nbkps; i++)
        {
            if (vkpstatus.at(i))
            {
                currFrame_->updateKeypoint(vkpids.at(i), vpriors.at(i));
                nbgood++;
            }
            else
            {
                // MapManager is responsible for all the removing operations
                mapManager_->removeObsFromCurrFrameById(vkpids.at(i));
            }
        }

        if (state_->debug_)
        {
            std::cout << "- [Visual-Front-End]: KLT Tracking no prior : " << nbgood << " out of " << nbkps << " kps tracked!\n";
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
    std::vector<bool> vkpis3d;

    // First we will track 3d kps on only 2 levels
    keyPoint3dIds.reserve(currFrame_->numKeypoints3d_);
    keyPoints3d.reserve(currFrame_->numKeypoints3d_);
    priors3d.reserve(currFrame_->numKeypoints3d_);

    // Then we'll track 2d kps on full pyramid levels
    keyPointIds.reserve(currFrame_->numKeypoints_);
    keyPoints.reserve(currFrame_->numKeypoints_);
    priors.reserve(currFrame_->numKeypoints_);

    vkpis3d.reserve(currFrame_->numKeypoints_);

    // Get prev keyframe
    auto pkf = mapManager_->mapKeyframes_.at(currFrame_->keyframeId_);

    if (pkf == nullptr)
    {
        return;
    }

    std::vector<int> badIds;
    badIds.reserve(currFrame_->numKeypoints_ * 0.2);

    // Front-End is thread-safe so we can direclty access curframe's kps
    for (const auto &it: currFrame_->mapKeypoints_)
    {
        auto &kp = it.second;
        auto kfkpit = pkf->mapKeypoints_.find(kp.keypointId_);

        if (kfkpit == pkf->mapKeypoints_.end())
        {
            badIds.push_back(kp.keypointId_);
            continue;
        }

        // Init prior px pos. from motion model
        if (state_->kltUsePrior_)
        {
            if (kp.is3d_)
            {
                cv::Point2f projpx = currFrame_->projWorldToImageDist(mapManager_->mapMapPoints_.at(kp.keypointId_)->getPoint());

                // Add prior if projected into image
                if (currFrame_->isInImage(projpx))
                {
                    keyPoints3d.push_back(kfkpit->second.px_);
                    priors3d.push_back(projpx);
                    keyPoint3dIds.push_back(kp.keypointId_);
                    vkpis3d.push_back(true);
                    continue;
                }
            }
        }

        // For other kps init prior with prev px pos.
        keyPointIds.push_back(kp.keypointId_);
        keyPoints.push_back(kfkpit->second.px_);
        priors.push_back(kp.px_);
    }

    for (const auto &badid: badIds)
    {
        // MapManager is responsible for all the removing operations
        mapManager_->removeObsFromCurrFrameById(badid);
    }

    // 1st track 3d kps if using prior
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

    // 2nd track other kps if any
    if (!keyPoints.empty())
    {
        // Good / bad kps vector
        std::vector<bool> vkpstatus;

        featureTracker_->fbKltTracking(
                keyframePyramid_,
                currPyramid_,
                state_->kltWinSizeWH_,
                state_->kltPyramidLevels_,
                state_->kltError_,
                state_->kltMaxFbDistance_,
                keyPoints,
                priors,
                vkpstatus);

        size_t nbgood = 0;
        size_t nbkps = keyPoints.size();

        for (size_t i = 0; i < nbkps; i++)
        {
            if (vkpstatus.at(i))
            {
                currFrame_->updateKeypoint(keyPointIds.at(i), priors.at(i));
                nbgood++;
            }
            else
            {
                // MapManager is responsible for all the removing operations
                mapManager_->removeObsFromCurrFrameById(keyPointIds.at(i));
            }
        }

        if (state_->debug_)
        {
            std::cout << "- [Visual-Front-End]: KLT Tracking no prior : " << nbgood << " out of " << nbkps << " kps tracked!\n";
        }
    }
}

void VisualFrontend::epipolar2d2dFiltering()
{
    // Get prev. keyframe (direct access as Front-End is thread safe)
    auto pkf = mapManager_->mapKeyframes_.at(currFrame_->keyframeId_);

    if (pkf == nullptr)
    {
        std::cerr << "- [Visual-Front-End]: ERROR! Previous Kf does not exist yet (epipolar2d2d()).\n";
        exit(-1);
    }

    // Get current frame number kps
    size_t numKeypoints = currFrame_->numKeypoints_;

    if (numKeypoints < 8)
    {
        if (state_->debug_)
        {
            std::cout << "- [Visual-Front-End]: Not enough kps to compute Essential Matrix\n";
        }
        return;
    }

    // Setup Essential Matrix computation for OpenGV-based filtering
    std::vector<int> vkpsids, voutliersidx;
    vkpsids.reserve(numKeypoints);
    voutliersidx.reserve(numKeypoints);

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > vkfbvs, vcurbvs;
    vkfbvs.reserve(numKeypoints);
    vcurbvs.reserve(numKeypoints);

    size_t numParallax = 0;
    float avgParallax = 0.;

    // Compute rotation compensated parallax
    Eigen::Matrix3d Rkfcur = pkf->getRcw() * currFrame_->getRwc();

    // Init bearing vectors and check parallax
    for (const auto &it: currFrame_->mapKeypoints_)
    {
        auto &kp = it.second;

        // Get the prev keyframe related kp if it exists
        auto kfkp = pkf->getKeypointById(kp.keypointId_);

        if (kfkp.keypointId_ != kp.keypointId_)
        {
            continue;
        }

        // Store the bvs and their ids
        vkfbvs.push_back(kfkp.bv_);
        vcurbvs.push_back(kp.bv_);
        vkpsids.push_back(kp.keypointId_);

        cv::Point2f rotPx = pkf->projCamToImage(Rkfcur * kp.bv_);

        // Compute parallax
        avgParallax += cv::norm(rotPx - kfkp.unpx_);
        numParallax++;
    }

    if (numKeypoints < 8)
    {
        if (state_->debug_)
        {
            std::cout << "- [Visual-Front-End]: Not enough kps to compute Essential Matrix\n";
        }
        return;
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

    bool do_optimize = false;

    // use the resulting motion if tracking is poor
    if (mapManager_->numKeyframes_ > 2 && currFrame_->numKeypoints3d_ < 30)
    {
        do_optimize = true;
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
            vkfbvs, vcurbvs,
            state_->multiViewRansacNumIterations_,
            state_->multiViewRansacError_,
            do_optimize,
            state_->multiViewRandomEnabled_,
            currFrame_->cameraCalibration_->fx_,
            currFrame_->cameraCalibration_->fy_,
            Rkfc, tkfc,
            voutliersidx);

    if (state_->debug_)
    {
        std::cout << "- [Visual-Front-End]: Epipolar nb outliers : " << voutliersidx.size();
    }

    if (!success)
    {
        if (state_->debug_)
        {
            std::cout << "- [Visual-Front-End]: No pose could be computed from 5-pt EssentialMatrix\n";
        }
        return;
    }

    if (voutliersidx.size() > 0.5 * vkfbvs.size())
    {
        if (state_->debug_)
        {
            std::cout << "- [Visual-Front-End]: Too many outliers, skipping as might be degenerate case\n";
        }
        return;
    }

    // Remove outliers
    for (const auto &idx: voutliersidx)
    {
        // MapManager is responsible for all the removing operations.
        mapManager_->removeObsFromCurrFrameById(vkpsids.at(idx));
    }

    // In case we wanted to use the resulting motion (can help when tracking is poor)
    if (do_optimize && mapManager_->numKeyframes_ > 2)
    {
        // Get motion model translation scale from last keyframe
        Sophus::SE3d Tkfw = pkf->getTcw();
        Sophus::SE3d Tkfcur = Tkfw * currFrame_->getTwc();

        double scale = Tkfcur.translation().norm();
        tkfc.normalize();

        // Update current pose with Essential Mat. relative motion
        // and current trans. scale
        Sophus::SE3d Tkfc(Rkfc, scale * tkfc);

        currFrame_->setTwc(pkf->getTwc() * Tkfc);
    }
}

bool VisualFrontend::computePose()
{
    size_t num3dKeypoints = currFrame_->numKeypoints3d_;

    if (num3dKeypoints < 4)
    {
        if (state_->debug_)
        {
            std::cout << "- [Visual-Front-End]: Not enough kps to compute P3P / PnP\n";
        }

        return false;
    }

    // setup P3P-Ransac computation for OpenGV-based Pose estimation + motion-only BA with Ceres
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > vbvs, vwpts;
    std::vector<int> vkpids;
    std::vector<int> voutliersidx;
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > vkps;

    vbvs.reserve(num3dKeypoints);
    vwpts.reserve(num3dKeypoints);
    vkpids.reserve(num3dKeypoints);
    voutliersidx.reserve(num3dKeypoints);
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
    bool do_optimize = false;
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
                vbvs, vwpts,
                state_->multiViewRansacNumIterations_,
                state_->multiViewRansacError_,
                do_optimize,
                state_->multiViewRandomEnabled_,
                currFrame_->cameraCalibration_->fx_,
                currFrame_->cameraCalibration_->fy_,
                Twc,
                voutliersidx,
                true //use meds, only effective with OpenGV
                );

        if (state_->debug_)
        {
            std::cout << "- [Visual-Front-End]: P3P-LMeds nb outliers : " << voutliersidx.size();
        }

        // Check that pose estimation was good enough
        size_t numInliers = vwpts.size() - voutliersidx.size();

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
        for (const auto &idx: voutliersidx)
        {
            // MapManager is responsible for all removing operations
            mapManager_->removeObsFromCurrFrameById(vkpids.at(idx - k));
            vkps.erase(vkps.begin() + idx - k);
            vwpts.erase(vwpts.begin() + idx - k);
            vkpids.erase(vkpids.begin() + idx - k);
            k++;
        }

        // Clear before robust PnP refinement using Ceres
        voutliersidx.clear();
    }

    // Ceres-based PnP (motion-only BA)
    bool useRobust = true;
    size_t maxIterations = 5;

    success = MultiViewGeometry::ceresPnP(
            vkps, vwpts,
            Twc,
            maxIterations,
            state_->baRobustThreshold_,
            useRobust,
            state_->applyL2AfterRobust_,
            currFrame_->cameraCalibration_->fx_, currFrame_->cameraCalibration_->fy_,
            currFrame_->cameraCalibration_->cx_, currFrame_->cameraCalibration_->cy_,
            voutliersidx);

    // Check that pose estimation was good enough
    size_t numInliers = vwpts.size() - voutliersidx.size();

    if (state_->debug_)
    {
        std::cout << "- [Visual-Front-End]: Ceres PnP outliers : " << voutliersidx.size() << ", inliers: " << numInliers << "\n";
    }

    if (!success || numInliers < 5 || voutliersidx.size() > 0.5 * vwpts.size() || Twc.translation().array().isInf().any() || Twc.translation().array().isNaN().any())
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

    // Remove outliers. MapManager is responsible for all removing operations
    for (const auto &idx: voutliersidx)
    {
        mapManager_->removeObsFromCurrFrameById(vkpids.at(idx));
    }

    return true;
}

bool VisualFrontend::checkReadyForInit()
{
    double avgComputedRotParallax = computeParallax(currFrame_->keyframeId_, false);

    if (avgComputedRotParallax > state_->minAvgRotationParallax_)
    {
        // get prev keyframe
        auto pkf = mapManager_->mapKeyframes_.at(currFrame_->keyframeId_);
        if (pkf == nullptr)
        {
            return false;
        }

        // get curr frame number of keypoints
        size_t numKeypoints = currFrame_->numKeypoints_;

        if (numKeypoints < 8)
        {
            std::cout << "- [Visual-Front-End]: Not enough keypoints to compute 5-pt essential matrix.\n";
            return false;
        }

        // Setup Essential Matrix computation for OpenGV-based filtering
        std::vector<int> vkpsids;
        std::vector<int> voutliersidx;
        vkpsids.reserve(numKeypoints);
        voutliersidx.reserve(numKeypoints);

        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > vkfbvs, vcurbvs;
        vkfbvs.reserve(numKeypoints);
        vcurbvs.reserve(numKeypoints);

        Eigen::Matrix3d Rkfcur = pkf->getTcw().rotationMatrix() * currFrame_->getTwc().rotationMatrix();
        int numParallax = 0;
        float avgRotParallax = 0.;

        // Get bvs and compute the rotation compensated parallax for all cur kps
        for (const auto &it: currFrame_->mapKeypoints_)
        {
            auto &kp = it.second;

            // get the prev keyframe related kp if it exists
            auto kfkp = pkf->getKeypointById(kp.keypointId_);

            if (kfkp.keypointId_ != kp.keypointId_)
            {
                continue;
            }

            // Store the bvs and their ids
            vkfbvs.push_back(kfkp.bv_);
            vcurbvs.push_back(kp.bv_);
            vkpsids.push_back(kp.keypointId_);

            // compute rotation compensated parallax
            Eigen::Vector3d rotBv = Rkfcur * kp.bv_;

            Eigen::Vector3d unpx = currFrame_->cameraCalibration_->K_ * rotBv;
            cv::Point2f rotpx(unpx.x() / unpx.z(), unpx.y() / unpx.z());

            avgRotParallax += cv::norm(rotpx - kfkp.unpx_);
            numParallax++;
        }

        if (numParallax < 8)
        {
            std::cout << "- [Visual-Front-End]: Not enough prev keyframe keypoints to compute 5-pt essential matrix.\n";
            return false;
        }

        // average parallax
        avgRotParallax /= float(numParallax);

        std::cout << "- [Visual-Front-End]: numParallax: " << numParallax << " Avg_rot_parallax:" << avgRotParallax << std::endl;

        if (avgRotParallax < state_->minAvgRotationParallax_)
        {
            std::cout << "- [Visual-Front-End]: Not enough parallax (" << avgRotParallax << " px) to compute 5-pt Essential Matrix\n";
            return false;
        }

        bool do_optimize = true;

        Eigen::Matrix3d Rkfc;
        Eigen::Vector3d tkfc;
        Rkfc.setIdentity();
        tkfc.setZero();

        bool success = MultiViewGeometry::compute5ptEssentialMatrix
                (vkfbvs, vcurbvs,
                 state_->multiViewRansacNumIterations_,
                 state_->multiViewRansacError_,
                 do_optimize,
                 state_->multiViewRandomEnabled_,
                 currFrame_->cameraCalibration_->fx_,
                 currFrame_->cameraCalibration_->fy_,
                 Rkfc, tkfc,
                 voutliersidx);

        if (!success)
        {
            std::cout << "- [Visual-Front-End]: No pose could be computed from 5-pt EssentialMatrix\n";
            return false;
        }

        // Remove outliers from cur. Frame
        for (const auto &idx: voutliersidx)
        {
            // MapManager is responsible for all the removing operations.
            mapManager_->removeObsFromCurrFrameById(vkpsids.at(idx));
        }

        // arbitrary scale
        tkfc.normalize();
        tkfc = tkfc.eval() * 0.25;

        currFrame_->setTwc(Rkfc, tkfc);

        auto ce = std::chrono::high_resolution_clock::now();

        return true;
    }

    return false;
}

bool VisualFrontend::checkNewKeyframeRequired()
{
    // get prev keyframe
    auto pkfit = mapManager_->mapKeyframes_.find(currFrame_->keyframeId_);

    if (pkfit == mapManager_->mapKeyframes_.end())
    {
        return false; // should not happen
    }

    auto pkf = pkfit->second;

    // Compute median parallax
    double med_rot_parallax = 0.0;

    // unrot : false / median : true / only_2d : false
    med_rot_parallax = computeParallax(pkf->keyframeId_, true, true, false);

    // Id diff with last keyframe
    int numImFromKeyframe = currFrame_->id_ - pkf->id_;

    if (currFrame_->numOccupiedCells_ < 0.33 * state_->frame_max_num_kps_ && numImFromKeyframe >= 5 && !state_->localBAActive_)
    {
        return true;
    }

    if (currFrame_->numKeypoints3d_ < 20 && numImFromKeyframe >= 2)
    {
        return true;
    }

    if (currFrame_->numKeypoints3d_ > 0.5 * state_->frame_max_num_kps_ && (state_->localBAActive_ || numImFromKeyframe < 2))
    {
        return false;
    }

    bool cx = med_rot_parallax >= state_->minAvgRotationParallax_ / 2.;

    bool c0 = med_rot_parallax >= state_->minAvgRotationParallax_;
    bool c1 = currFrame_->numKeypoints3d_ < 0.75 * pkf->numKeypoints3d_;
    bool c2 = currFrame_->numOccupiedCells_ < 0.5 * state_->frame_max_num_kps_ && currFrame_->numKeypoints3d_ < 0.85 * pkf->numKeypoints3d_ && !state_->localBAActive_;

    bool keyFrameRequired = (c0 || c1 || c2) && cx;

    if (keyFrameRequired && state_->debug_)
    {
        std::cout << "\n\n----------------------------------------------------------------------";
        std::cout << "\n>>> Check Keyframe conditions :";
        std::cout << "\n> curr_frame_->id_ = " << currFrame_->id_ << " / prev kf frame_id : " << pkf->id_;
        std::cout << "\n> prev keyframe nb 3d kps = " << pkf->numKeypoints3d_ << " / Cur Frame = " << currFrame_->numKeypoints3d_;
        std::cout << " / curr frame occupied cells = " << currFrame_->numOccupiedCells_ << " / parallax = " << med_rot_parallax;
        std::cout << "\n-------------------------------------------------------------------\n\n";
    }

    return keyFrameRequired;
}

float VisualFrontend::computeParallax(const int keyframeId, bool doUnRotate, bool doMedian, bool doOnly2d)
{
    // Get prev keyframe
    auto keyframes = mapManager_->mapKeyframes_.find(keyframeId);

    if (keyframes == mapManager_->mapKeyframes_.end())
    {
        if (state_->debug_)
        {
            std::cout << "- [Visual-Front-End]: Error in computeParallax ! Prev keyframe #" << keyframeId << " does not exist!\n";
        }

        return 0.;
    }

    // Compute relative rotation between cur Frame and prev. keyframe if required
    Eigen::Matrix3d Rkfcur(Eigen::Matrix3d::Identity());

    if (doUnRotate)
    {
        Eigen::Matrix3d Rkfw = keyframes->second->getRcw();
        Eigen::Matrix3d Rwcur = currFrame_->getRwc();
        Rkfcur = Rkfw * Rwcur;
    }

    // Compute parallax
    float avgParallax = 0.;
    int numParallax = 0;

    std::set<float> parallaxSet;

    // Compute parallax for all kps seen in prev. keyframe{
    for (const auto &it: currFrame_->mapKeypoints_)
    {
        if (doOnly2d && it.second.is3d_)
        {
            continue;
        }

        auto &kp = it.second;
        // Get prev. keyframe kp if it exists
        auto keypoint = keyframes->second->getKeypointById(kp.keypointId_);

        if (keypoint.keypointId_ != kp.keypointId_)
        {
            continue;
        }

        // Compute parallax with unpx position
        cv::Point2f unpx = kp.unpx_;

        // Rotate bv into keyframe cam frame and back project into image
        if (doUnRotate)
        {
            unpx = keyframes->second->projCamToImage(Rkfcur * kp.bv_);
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

void VisualFrontend::preprocessImage(cv::Mat &imageRaw)
{
    // update prev image
    if (!state_->trackKeyframeToFrame_)
    {
        cv::swap(currImage_, prevImage_);
    }

    // update curr image
    if (state_->claheEnabled_)
    {
        clahe_->apply(imageRaw, currImage_);
    }
    else
    {
        currImage_ = imageRaw;
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
}
