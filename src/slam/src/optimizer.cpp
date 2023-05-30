#include "optimizer.hpp"
#include "ceres_parametrization.hpp"

void Optimizer::localBA(Frame &newFrame)
{
    const bool useRobustCost = true;
    const float chi2errThreshold = state_->robustCostThreshold_;
    const bool applyL2AfterRobust = state_->robustCostRefineWithL2_;
    const bool inverseDepthEnabled = state_->baInverseDepthEnabled_;
    const int minCovScore = state_->baMinNumCommonKeypointsObservations_;

    // Do not optimize if tracking is poor
    if ((int) newFrame.numKeypoints3d_ < minCovScore)
    {
        return;
    }

    // ================================================================== 1. Setup BA Problem

    ceres::Problem problem;
    ceres::LossFunctionWrapper *lossFunction;
    lossFunction = new ceres::LossFunctionWrapper(new ceres::HuberLoss(std::sqrt(chi2errThreshold)), ceres::TAKE_OWNERSHIP);

    if (!useRobustCost)
    {
        lossFunction->Reset(nullptr, ceres::TAKE_OWNERSHIP);
    }

    auto ordering = new ceres::ParameterBlockOrdering;

    std::unordered_map<int, PoseParametersBlock> map_id_posespar_;
    std::unordered_map<int, PointXYZParametersBlock> map_id_pointspar_;
    std::unordered_map<int, InvDepthParametersBlock> map_id_invptspar_;

    std::unordered_map<int, std::shared_ptr<MapPoint>> map_local_plms;
    std::unordered_map<int, std::shared_ptr<Frame>> map_local_pkfs;

    // Storing the factors and their residuals block ids for fast accessing when checking for outliers
    std::vector<std::pair<ceres::CostFunction *, std::pair<ceres::ResidualBlockId, std::pair<int, int>>>> vreprojerr_kfid_lmid;
    std::vector<std::pair<ceres::CostFunction *, std::pair<ceres::ResidualBlockId, std::pair<int, int>>>> vright_reprojerr_kfid_lmid;

    // Add the cam calibration parameters
    auto cameraCalibration = newFrame.cameraCalibration_;
    CalibParametersBlock calibParams(0, cameraCalibration->fx_, cameraCalibration->fy_, cameraCalibration->cx_, cameraCalibration->cy_);
    problem.AddParameterBlock(calibParams.values(), 4);
    ordering->AddElementToGroup(calibParams.values(), 1);

    problem.SetParameterBlockConstant(calibParams.values());

    // Get the new keyframe covisible keyframes
    std::map<int, int> covisibleKeyframeMap = newFrame.getCovisibleKeyframeMap();

    // Add the new keyframe to the map with max score
    covisibleKeyframeMap.emplace(newFrame.keyframeId_, newFrame.numKeypoints3d_);

    // Keep track of map points not suited for BA for speed-up
    std::unordered_set<int> set_badlmids;
    std::unordered_set<int> set_lmids2opt;
    std::unordered_set<int> set_kfids2opt;
    std::unordered_set<int> set_cstkfids;

    bool all_cst = false;

    // Go through the covisibility keyframe map
    int nmaxkfid = covisibleKeyframeMap.rbegin()->first;

    for (auto it = covisibleKeyframeMap.rbegin(); it != covisibleKeyframeMap.rend(); it++)
    {
        int keyframeId = it->first;
        int covScore = it->second;

        if (keyframeId > newFrame.keyframeId_)
        {
            covScore = newFrame.numKeypoints_;
        }

        auto pkf = mapManager_->getKeyframe(keyframeId);

        if (pkf == nullptr)
        {
            newFrame.removeCovisibleKeyframe(keyframeId);
            continue;
        }

        // Add every keyframe to BA problem
        map_id_posespar_.emplace(keyframeId, PoseParametersBlock(keyframeId, pkf->getTwc()));

        ceres::LocalParameterization *local_parameterization = new SE3Parameterization();

        problem.AddParameterBlock(map_id_posespar_.at(keyframeId).values(), 7, local_parameterization);
        ordering->AddElementToGroup(map_id_posespar_.at(keyframeId).values(), 1);

        // For those to optimize, get their 3D map points for the others, set them as constant
        if (covScore >= minCovScore && !all_cst && keyframeId > 0)
        {
            set_kfids2opt.insert(keyframeId);
            for (const auto &kp: pkf->getKeypoints3d())
            {
                set_lmids2opt.insert(kp.keypointId_);
            }
        }
        else
        {
            set_cstkfids.insert(keyframeId);
            problem.SetParameterBlockConstant(map_id_posespar_.at(keyframeId).values());
            all_cst = true;
        }

        map_local_pkfs.emplace(keyframeId, pkf);
    }

    // Go through the map points to optimize
    for (const auto &lmid: set_lmids2opt)
    {
        auto plm = mapManager_->getMapPoint(lmid);

        if (plm == nullptr)
        {
            continue;
        }

        if (plm->isBad())
        {
            set_badlmids.insert(lmid);
            continue;
        }

        map_local_plms.emplace(lmid, plm);

        if (!inverseDepthEnabled)
        {
            map_id_pointspar_.emplace(lmid, PointXYZParametersBlock(lmid, plm->getPoint()));

            problem.AddParameterBlock(map_id_pointspar_.at(lmid).values(), 3);
            ordering->AddElementToGroup(map_id_pointspar_.at(lmid).values(), 0);
        }

        int kfanchid = -1;
        double unanch_u = -1.;
        double unanch_v = -1.;

        for (const auto &kfid: plm->getObservedKeyframeIds())
        {
            if (kfid > nmaxkfid)
            {
                continue;
            }

            auto pkfit = map_local_pkfs.find(kfid);
            std::shared_ptr<Frame> pkf = nullptr;

            // Add the observing keyframe if not set yet
            if (pkfit == map_local_pkfs.end())
            {
                pkf = mapManager_->getKeyframe(kfid);
                if (pkf == nullptr)
                {
                    mapManager_->removeMapPointObs(kfid, plm->mapPointId_);
                    continue;
                }
                map_local_pkfs.emplace(kfid, pkf);
                map_id_posespar_.emplace(kfid, PoseParametersBlock(kfid, pkf->getTwc()));

                ceres::LocalParameterization *local_parameterization = new SE3Parameterization();

                problem.AddParameterBlock(map_id_posespar_.at(kfid).values(), 7, local_parameterization);
                ordering->AddElementToGroup(map_id_posespar_.at(kfid).values(), 1);

                set_cstkfids.insert(kfid);
                problem.SetParameterBlockConstant(map_id_posespar_.at(kfid).values());

            }
            else
            {
                pkf = pkfit->second;
            }

            auto kp = pkf->getKeypointById(lmid);

            if (kp.keypointId_ != lmid)
            {
                mapManager_->removeMapPointObs(lmid, kfid);
                continue;
            }

            if (inverseDepthEnabled)
            {
                if (kfanchid < 0)
                {
                    kfanchid = kfid;
                    unanch_u = kp.unpx_.x;
                    unanch_v = kp.unpx_.y;
                    double zanch = (pkf->getTcw() * plm->getPoint()).z();
                    map_id_invptspar_.emplace(lmid, InvDepthParametersBlock(lmid, kfanchid, zanch));

                    problem.AddParameterBlock(map_id_invptspar_.at(lmid).values(), 1);
                    ordering->AddElementToGroup(map_id_invptspar_.at(lmid).values(), 0);

                    continue;
                }
            }

            ceres::CostFunction *f;
            ceres::ResidualBlockId rid;

            // Add a visual factor between keyframe-map point nodes
            if (inverseDepthEnabled)
            {
                const float scale = 1.0;

                f = new DirectSE3::ReprojectionErrorKSE3AnchInvDepth(kp.unpx_.x, kp.unpx_.y, unanch_u, unanch_v, scale);

                rid = problem.AddResidualBlock(
                        f, lossFunction,
                        calibParams.values(),
                        map_id_posespar_.at(kfanchid).values(),
                        map_id_posespar_.at(kfid).values(),
                        map_id_invptspar_.at(lmid).values()
                );
            }
            else
            {
                const float scale = 1.0;

                f = new DirectSE3::ReprojectionErrorKSE3XYZ(kp.unpx_.x, kp.unpx_.y, scale);

                rid = problem.AddResidualBlock(f, lossFunction, calibParams.values(), map_id_posespar_.at(kfid).values(), map_id_pointspar_.at(lmid).values());
            }

            vreprojerr_kfid_lmid.push_back(std::make_pair(f, std::make_pair(rid, std::make_pair(kfid, kp.keypointId_))));
        }
    }

    // Ensure the gauge is fixed
    size_t nbcstkfs = set_cstkfids.size();

    // At least two fixed keyframe
    size_t nmincstkfs = 2;
    if (nbcstkfs < nmincstkfs)
    {
        for (auto it = map_local_pkfs.begin(); nbcstkfs < nmincstkfs && it != map_local_pkfs.end(); it++)
        {
            problem.SetParameterBlockConstant(map_id_posespar_.at(it->first).values());
            set_cstkfids.insert(it->first);
            nbcstkfs++;
        }
    }

    // ================================================================== 2. Solve BA Problem

    ceres::Solver::Options options;
    options.linear_solver_ordering.reset(ordering);
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.num_threads = 1;
    options.max_num_iterations = 5;
    options.function_tolerance = 0.001;
    options.max_solver_time_in_seconds = 0.01; //10ms
    options.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // ================================================================== 3. Remove outliers

    size_t numBadObservations = 0;

    std::vector<std::pair<int, int>> badKeyframeLandmarkIds;
    badKeyframeLandmarkIds.reserve(vreprojerr_kfid_lmid.size() / 10);

    for (auto it = vreprojerr_kfid_lmid.begin(); it != vreprojerr_kfid_lmid.end();)
    {
        bool bigChi2 = true;
        bool depthPositive = true;

        if (inverseDepthEnabled)
        {
            auto *err = static_cast<DirectSE3::ReprojectionErrorKSE3AnchInvDepth *>(it->first);
            bigChi2 = err->chi2err_ > chi2errThreshold;
            depthPositive = err->isDepthPositive_;
        }
        else
        {
            auto *err = static_cast<DirectSE3::ReprojectionErrorKSE3XYZ *>(it->first);
            bigChi2 = err->chi2err_ > chi2errThreshold;
            depthPositive = err->isDepthPositive_;
        }

        if (bigChi2 || !depthPositive)
        {
            if (applyL2AfterRobust)
            {
                auto rid = it->second.first;
                problem.RemoveResidualBlock(rid);
            }

            int lmid = it->second.second.second;
            int kfid = it->second.second.first;
            badKeyframeLandmarkIds.push_back(std::pair<int, int>(kfid, lmid));
            set_badlmids.insert(lmid);
            numBadObservations++;

            it = vreprojerr_kfid_lmid.erase(it);
        }
        else
        {
            it++;
        }
    }

    // ================================================================== 4. Refine BA Solution

    if (applyL2AfterRobust && useRobustCost && numBadObservations > 0)
    {
        if (!vreprojerr_kfid_lmid.empty() && !vright_reprojerr_kfid_lmid.empty())
        {
            lossFunction->Reset(nullptr, ceres::TAKE_OWNERSHIP);
        }

        options.max_num_iterations = 5;
        options.function_tolerance = 0.001;
        options.max_solver_time_in_seconds = 0.001; //1ms

        ceres::Solve(options, &problem, &summary);

        // Remove bad observations
        for (auto it = vreprojerr_kfid_lmid.begin(); it != vreprojerr_kfid_lmid.end();)
        {
            bool bigChi2 = true;
            bool depthPositive = true;

            if (inverseDepthEnabled)
            {
                auto *err = static_cast<DirectSE3::ReprojectionErrorKSE3AnchInvDepth *>(it->first);
                bigChi2 = err->chi2err_ > chi2errThreshold;
                depthPositive = err->isDepthPositive_;
            }
            else
            {
                auto *err = static_cast<DirectSE3::ReprojectionErrorKSE3XYZ *>(it->first);
                bigChi2 = err->chi2err_ > chi2errThreshold;
                depthPositive = err->isDepthPositive_;
            }

            if (bigChi2 || !depthPositive)
            {
                int mapPointId = it->second.second.second;
                int keyframeId = it->second.second.first;
                badKeyframeLandmarkIds.push_back(std::pair<int, int>(keyframeId, mapPointId));
                set_badlmids.insert(mapPointId);

                it = vreprojerr_kfid_lmid.erase(it);
            }
            else
            {
                it++;
            }
        }
    }

    // ================================================================== 5. Update State Parameters

    for (const auto &pair: badKeyframeLandmarkIds)
    {
        int kfId = pair.first;
        int lmId = pair.second;
        auto it = map_local_pkfs.find(kfId);

        if (it != map_local_pkfs.end())
        {
            mapManager_->removeMapPointObs(lmId, kfId);
        }

        if (kfId == mapManager_->currFrame_->keyframeId_)
        {
            mapManager_->removeObsFromCurrFrameById(lmId);
        }

        set_badlmids.insert(lmId);
    }

    // Update keyframes
    for (const auto &pair: map_local_pkfs)
    {
        int kfId = pair.first;

        if (set_cstkfids.count(kfId))
        {
            continue;
        }

        auto keyframe = pair.second;

        if (keyframe == nullptr)
        {
            continue;
        }

        auto it = map_id_posespar_.find(kfId);

        if (it != map_id_posespar_.end())
        {
            keyframe->setTwc(it->second.getPose());
        }
    }

    // Update map points
    for (const auto &pair: map_local_plms)
    {
        int lmId = pair.first;
        auto mapPoint = pair.second;

        if (mapPoint == nullptr)
        {
            set_badlmids.erase(lmId);
            continue;
        }

        if (mapPoint->isBad())
        {
            mapManager_->removeMapPoint(lmId);
            set_badlmids.erase(lmId);
            continue;
        }

        // Map Point Culling
        auto kfids = mapPoint->getObservedKeyframeIds();

        if (kfids.size() < 3)
        {
            if (mapPoint->keyframeId_ < newFrame.keyframeId_ - 3 && !mapPoint->isObserved_)
            {
                mapManager_->removeMapPoint(lmId);
                set_badlmids.erase(lmId);
                continue;
            }
        }

        if (inverseDepthEnabled)
        {
            auto invptit = map_id_invptspar_.find(lmId);

            if (invptit == map_id_invptspar_.end())
            {
                set_badlmids.insert(lmId);
                continue;
            }

            double zanch = 1. / invptit->second.getInvDepth();
            if (zanch <= 0.)
            {
                mapManager_->removeMapPoint(lmId);
                set_badlmids.erase(lmId);
                continue;
            }

            auto it = map_local_pkfs.find(mapPoint->keyframeId_);
            if (it == map_local_pkfs.end())
            {
                set_badlmids.insert(lmId);
                continue;
            }

            auto pkfanch = it->second;
            if (pkfanch != nullptr)
            {
                auto kp = pkfanch->getKeypointById(lmId);
                Eigen::Vector3d uvpt(kp.unpx_.x, kp.unpx_.y, 1.);
                Eigen::Vector3d optwpt = pkfanch->getTwc() * (zanch * pkfanch->cameraCalibration_->inverseK_ * uvpt);
                mapManager_->updateMapPoint(lmId, optwpt, invptit->second.getInvDepth());
            }
            else
            {
                set_badlmids.insert(lmId);
            }
        }
        else
        {
            auto optlmit = map_id_pointspar_.find(lmId);
            if (optlmit != map_id_pointspar_.end())
            {
                mapManager_->updateMapPoint(lmId, optlmit->second.getPoint());
            }
            else
            {
                set_badlmids.insert(lmId);
            }
        }
    }

    // Map point culling for bad observations
    size_t numBadLandmarks = 0;
    for (const auto &lmid: set_badlmids)
    {
        std::shared_ptr<MapPoint> mapPoint;
        auto plmit = map_local_plms.find(lmid);

        if (plmit == map_local_plms.end())
        {
            mapPoint = mapManager_->getMapPoint(lmid);
        }
        else
        {
            mapPoint = plmit->second;
        }

        if (mapPoint == nullptr)
        {
            continue;
        }

        if (mapPoint->isBad())
        {
            mapManager_->removeMapPoint(lmid);
            numBadLandmarks++;
        }
        else
        {
            auto set_cokfs = mapPoint->getObservedKeyframeIds();

            if (set_cokfs.size() < 3)
            {
                if (mapPoint->keyframeId_ < newFrame.keyframeId_ - 3 && !mapPoint->isObserved_)
                {
                    mapManager_->removeMapPoint(lmid);
                    numBadLandmarks++;
                }
            }
        }
    }
}
