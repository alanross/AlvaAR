#include "optimizer.hpp"
#include "ceres_parametrization.hpp"

void Optimizer::localBA(Frame &newFrame, const bool useRobustCost)
{
    // =================================
    //      Setup BA Problem
    // =================================

    ceres::Problem problem;
    ceres::LossFunctionWrapper *lossFunction;

    // Chi2 thresh.
    const float threshold = state_->baRobustThreshold_;

    lossFunction = new ceres::LossFunctionWrapper(new ceres::HuberLoss(std::sqrt(threshold)), ceres::TAKE_OWNERSHIP);

    if (!useRobustCost)
    {
        lossFunction->Reset(nullptr, ceres::TAKE_OWNERSHIP);
    }

    // Thresh. score for optimizing / fixing a keyframe in BA (cov score with new keyframe)
    int minCovScore = state_->localBAMinNumCommonKeypointsObservations;

    // Do not optimize if tracking is poor (hopefully will get better soon!)
    if ((int) newFrame.numKeypoints3d_ < minCovScore)
    {
        return;
    }

    size_t nmincstkfs = 2;

    size_t numMono = 0;

    auto ordering = new ceres::ParameterBlockOrdering;

    std::unordered_map<int, PoseParametersBlock> map_id_posespar_;
    std::unordered_map<int, PointXYZParametersBlock> map_id_pointspar_;
    std::unordered_map<int, InvDepthParametersBlock> map_id_invptspar_;

    std::unordered_map<int, std::shared_ptr<MapPoint>> map_local_plms;
    std::unordered_map<int, std::shared_ptr<Frame>> map_local_pkfs;

    // Storing the factors and their residuals block ids for fast accessing when checking for outliers
    std::vector<std::pair<ceres::CostFunction *, std::pair<ceres::ResidualBlockId, std::pair<int, int>>>> vreprojerr_kfid_lmid, vright_reprojerr_kfid_lmid;

    // Add the cam calibration parameters
    auto pcalib_cam = newFrame.cameraCalibration_;
    CalibParametersBlock calibpar(0, pcalib_cam->fx_, pcalib_cam->fy_, pcalib_cam->cx_, pcalib_cam->cy_);
    problem.AddParameterBlock(calibpar.values(), 4);
    ordering->AddElementToGroup(calibpar.values(), 1);

    problem.SetParameterBlockConstant(calibpar.values());

    Sophus::SE3d Trl, Tlr;
    PoseParametersBlock rlextrinpose(0, Trl);

    // Get the new keyframe covisible keyframes
    std::map<int, int> map_covkfs = newFrame.getCovisibleKeyframeMap();

    // Add the new keyframe to the map with max score
    map_covkfs.emplace(newFrame.keyframeId_, newFrame.numKeypoints3d_);

    // Keep track of map points no suited for BA for speed-up
    std::unordered_set<int> set_badlmids;
    std::unordered_set<int> set_lmids2opt;
    std::unordered_set<int> set_kfids2opt;
    std::unordered_set<int> set_cstkfids;

    if (state_->debug_)
    {
        std::cout << "\n >>> Local BA : new keyframe is #" << newFrame.keyframeId_ << " -- with covisible graph of size : " << map_covkfs.size();
    }

    bool all_cst = false;

    // Go through the covisibility Kf map
    int nmaxkfid = map_covkfs.rbegin()->first;

    for (auto it = map_covkfs.rbegin(); it != map_covkfs.rend(); it++)
    {

        int kfid = it->first;
        int covscore = it->second;

        if (kfid > newFrame.keyframeId_)
        {
            covscore = newFrame.numKeypoints_;
        }

        auto pkf = mapManager_->getKeyframe(kfid);

        if (pkf == nullptr)
        {
            newFrame.removeCovisibleKeyframe(kfid);
            continue;
        }

        // Add every keyframe to BA problem
        map_id_posespar_.emplace(kfid, PoseParametersBlock(kfid, pkf->getTwc()));

        ceres::LocalParameterization *local_parameterization = new SE3Parameterization();

        problem.AddParameterBlock(map_id_posespar_.at(kfid).values(), 7, local_parameterization);
        ordering->AddElementToGroup(map_id_posespar_.at(kfid).values(), 1);

        // For those to optimize, get their 3D map points for the others, set them as constant
        if (covscore >= minCovScore && !all_cst && kfid > 0)
        {
            set_kfids2opt.insert(kfid);
            for (const auto &kp: pkf->getKeypoints3d())
            {
                set_lmids2opt.insert(kp.keypointId_);
            }
        }
        else
        {
            set_cstkfids.insert(kfid);
            problem.SetParameterBlockConstant(map_id_posespar_.at(kfid).values());
            all_cst = true;
        }

        map_local_pkfs.emplace(kfid, pkf);
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

        if (!state_->inverseDepthEnabled_)
        {
            map_id_pointspar_.emplace(lmid, PointXYZParametersBlock(lmid, plm->getPoint()));

            problem.AddParameterBlock(map_id_pointspar_.at(lmid).values(), 3);
            ordering->AddElementToGroup(map_id_pointspar_.at(lmid).values(), 0);
        }

        int kfanchid = -1;
        double unanch_u = -1.;
        double unanch_v = -1.;

        for (const auto &kfid: plm->getKeyframeObsSet())
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

            if (state_->inverseDepthEnabled_)
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

                    numMono++;

                    continue;
                }
            }

            ceres::CostFunction *f;
            ceres::ResidualBlockId rid;

            // Add a visual factor between keyframe-map point nodes
            if (state_->inverseDepthEnabled_)
            {
                const float scale = 1.0;

                f = new DirectSE3::ReprojectionErrorKSE3AnchInvDepth(kp.unpx_.x, kp.unpx_.y, unanch_u, unanch_v, scale);

                rid = problem.AddResidualBlock(
                        f, lossFunction,
                        calibpar.values(),
                        map_id_posespar_.at(kfanchid).values(),
                        map_id_posespar_.at(kfid).values(),
                        map_id_invptspar_.at(lmid).values()
                );
            }
            else
            {
                const float scale = 1.0;

                f = new DirectSE3::ReprojectionErrorKSE3XYZ(kp.unpx_.x, kp.unpx_.y, scale);

                rid = problem.AddResidualBlock(f, lossFunction, calibpar.values(), map_id_posespar_.at(kfid).values(), map_id_pointspar_.at(lmid).values());
            }

            vreprojerr_kfid_lmid.push_back(std::make_pair(f, std::make_pair(rid, std::make_pair(kfid, kp.keypointId_))));

            numMono++;
        }
    }

    // Ensure the gauge is fixed
    size_t nbcstkfs = set_cstkfids.size();

    // At least two fixed keyframe
    if (nbcstkfs < nmincstkfs)
    {
        for (auto it = map_local_pkfs.begin(); nbcstkfs < nmincstkfs && it != map_local_pkfs.end(); it++)
        {
            problem.SetParameterBlockConstant(map_id_posespar_.at(it->first).values());
            set_cstkfids.insert(it->first);
            nbcstkfs++;
        }
    }

    size_t nbkfstot = map_local_pkfs.size();
    size_t nbkfs2opt = nbkfstot - nbcstkfs;
    size_t nblms2opt = map_local_plms.size();

    if (state_->debug_)
    {
        std::cout << "\n\n >>> LocalBA problem setup!";
        std::cout << "\n >>> Kfs added (opt / tot) : " << nbkfs2opt << " / " << nbkfstot;
        std::cout << "\n >>> map points added : " << nblms2opt;
        std::cout << "\n >>> Measurements added : " << numMono;

        std::cout << "\n\n >>> keyframes added : ";
        for (const auto &id_pkf: map_local_pkfs)
        {
            std::cout << " keyframe #" << id_pkf.first << " (cst : " << set_cstkfids.count(id_pkf.first) << "), ";
        }

        std::cout << "\n\n >>> keyframes in cov map : ";
        for (const auto &id_score: map_covkfs)
        {
            std::cout << " keyframe #" << id_score.first << " (cov score : " << id_score.second << "), ";
        }
    }

    // =================================
    //      Solve BA Problem
    // =================================

    ceres::Solver::Options options;
    options.linear_solver_ordering.reset(ordering);
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.num_threads = 1;
    options.max_num_iterations = 5;
    options.function_tolerance = 1.e-3;
    options.max_solver_time_in_seconds = 0.2;
    options.minimizer_progress_to_stdout = state_->debug_;
    options.max_solver_time_in_seconds *= 2.; //set only if not forcing realtime

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if (state_->debug_)
    {
        std::cout << summary.FullReport() << std::endl;
    }

    // =================================
    //      Remove outliers
    // =================================

    size_t nbbadobsmono = 0;

    std::vector<std::pair<int, int>> vbadkflmids;
    vbadkflmids.reserve(vreprojerr_kfid_lmid.size() / 10);

    for (auto it = vreprojerr_kfid_lmid.begin(); it != vreprojerr_kfid_lmid.end();)
    {
        bool bbigchi2 = true;
        bool bdepthpos = true;

        if (state_->inverseDepthEnabled_)
        {
            auto *err = static_cast<DirectSE3::ReprojectionErrorKSE3AnchInvDepth *>(it->first);
            bbigchi2 = err->chi2err_ > threshold;
            bdepthpos = err->isDepthPositive_;
        }
        else
        {
            auto *err = static_cast<DirectSE3::ReprojectionErrorKSE3XYZ *>(it->first);
            bbigchi2 = err->chi2err_ > threshold;
            bdepthpos = err->isDepthPositive_;
        }

        if (bbigchi2 || !bdepthpos)
        {
            if (state_->applyL2AfterRobust_)
            {
                auto rid = it->second.first;
                problem.RemoveResidualBlock(rid);
            }
            int lmid = it->second.second.second;
            int kfid = it->second.second.first;
            vbadkflmids.push_back(std::pair<int, int>(kfid, lmid));
            set_badlmids.insert(lmid);
            nbbadobsmono++;

            it = vreprojerr_kfid_lmid.erase(it);
        }
        else
        {
            it++;
        }
    }

    size_t nbbadobs = nbbadobsmono;

    // =================================
    //      Refine BA Solution
    // =================================

    bool bl2optimdone = false;

    // Refine without Robust cost if req.
    if (state_->applyL2AfterRobust_ && useRobustCost && nbbadobs > 0)
    {
        if (!vreprojerr_kfid_lmid.empty() && !vright_reprojerr_kfid_lmid.empty())
        {
            lossFunction->Reset(nullptr, ceres::TAKE_OWNERSHIP);
        }

        options.max_num_iterations = 10;
        options.function_tolerance = 1.e-3;
        options.max_solver_time_in_seconds /= 2.;

        ceres::Solve(options, &problem, &summary);

        bl2optimdone = true;

        if (state_->debug_)
        {
            std::cout << summary.FullReport() << std::endl;
        }
    }

    // =================================
    //      Remove outliers
    // =================================

    // Remove Bad Observations
    if (bl2optimdone)
    {
        for (auto it = vreprojerr_kfid_lmid.begin(); it != vreprojerr_kfid_lmid.end();)
        {
            bool bbigchi2 = true;
            bool bdepthpos = true;

            if (state_->inverseDepthEnabled_)
            {
                auto *err = static_cast<DirectSE3::ReprojectionErrorKSE3AnchInvDepth *>(it->first);
                bbigchi2 = err->chi2err_ > threshold;
                bdepthpos = err->isDepthPositive_;
            }
            else
            {
                auto *err = static_cast<DirectSE3::ReprojectionErrorKSE3XYZ *>(it->first);
                bbigchi2 = err->chi2err_ > threshold;
                bdepthpos = err->isDepthPositive_;
            }

            if (bbigchi2 || !bdepthpos)
            {
                int lmid = it->second.second.second;
                int kfid = it->second.second.first;
                vbadkflmids.push_back(std::pair<int, int>(kfid, lmid));
                set_badlmids.insert(lmid);
                nbbadobsmono++;

                it = vreprojerr_kfid_lmid.erase(it);
            }
            else
            {
                it++;
            }
        }
    }

    // =================================
    //      Update State Parameters
    // =================================

    for (const auto &badkflmid: vbadkflmids)
    {
        int kfid = badkflmid.first;
        int lmid = badkflmid.second;
        auto it = map_local_pkfs.find(kfid);
        if (it != map_local_pkfs.end())
        {
            mapManager_->removeMapPointObs(lmid, kfid);
        }
        if (kfid == mapManager_->currFrame_->keyframeId_)
        {
            mapManager_->removeObsFromCurrFrameById(lmid);
        }
        set_badlmids.insert(lmid);
    }

    // Update keyframes
    for (const auto &kfid_pkf: map_local_pkfs)
    {
        int kfid = kfid_pkf.first;

        if (set_cstkfids.count(kfid))
        {
            continue;
        }

        auto pkf = kfid_pkf.second;

        if (pkf == nullptr)
        {
            continue;
        }

        auto it = map_id_posespar_.find(kfid);

        if (it != map_id_posespar_.end())
        {
            pkf->setTwc(it->second.getPose());
        }
    }

    // Update map points
    for (const auto &lmid_plm: map_local_plms)
    {
        int lmid = lmid_plm.first;
        auto plm = lmid_plm.second;

        if (plm == nullptr)
        {
            set_badlmids.erase(lmid);
            continue;
        }

        if (plm->isBad())
        {
            mapManager_->removeMapPoint(lmid);
            set_badlmids.erase(lmid);
            continue;
        }

        // Map Point Culling
        auto kfids = plm->getKeyframeObsSet();
        if (kfids.size() < 3)
        {
            if (plm->keyframeId_ < newFrame.keyframeId_ - 3 && !plm->isObserved_)
            {
                mapManager_->removeMapPoint(lmid);
                set_badlmids.erase(lmid);
                continue;
            }
        }

        if (state_->inverseDepthEnabled_)
        {
            auto invptit = map_id_invptspar_.find(lmid);
            if (invptit == map_id_invptspar_.end())
            {
                set_badlmids.insert(lmid);
                continue;
            }
            double zanch = 1. / invptit->second.getInvDepth();
            if (zanch <= 0.)
            {
                mapManager_->removeMapPoint(lmid);
                set_badlmids.erase(lmid);
                continue;
            }

            auto it = map_local_pkfs.find(plm->keyframeId_);
            if (it == map_local_pkfs.end())
            {
                set_badlmids.insert(lmid);
                continue;
            }
            auto pkfanch = it->second;

            if (pkfanch != nullptr)
            {
                auto kp = pkfanch->getKeypointById(lmid);
                Eigen::Vector3d uvpt(kp.unpx_.x, kp.unpx_.y, 1.);
                Eigen::Vector3d optwpt = pkfanch->getTwc() * (zanch * pkfanch->cameraCalibration_->inverseK_ * uvpt);
                mapManager_->updateMapPoint(lmid, optwpt, invptit->second.getInvDepth());
            }
            else
            {
                set_badlmids.insert(lmid);
            }
        }
        else
        {
            auto optlmit = map_id_pointspar_.find(lmid);
            if (optlmit != map_id_pointspar_.end())
            {
                mapManager_->updateMapPoint(lmid, optlmit->second.getPoint());
            }
            else
            {
                set_badlmids.insert(lmid);
            }
        }
    }

    // Map Point Culling for bad Obs.
    size_t nbbadlm = 0;
    for (const auto &lmid: set_badlmids)
    {
        std::shared_ptr<MapPoint> plm;
        auto plmit = map_local_plms.find(lmid);
        if (plmit == map_local_plms.end())
        {
            plm = mapManager_->getMapPoint(lmid);
        }
        else
        {
            plm = plmit->second;
        }
        if (plm == nullptr)
        {
            continue;
        }

        if (plm->isBad())
        {
            mapManager_->removeMapPoint(lmid);
            nbbadlm++;
        }
        else
        {
            auto set_cokfs = plm->getKeyframeObsSet();
            if (set_cokfs.size() < 3)
            {
                if (plm->keyframeId_ < newFrame.keyframeId_ - 3 && !plm->isObserved_)
                {
                    mapManager_->removeMapPoint(lmid);
                    nbbadlm++;
                }
            }
        }
    }

    nbbadobs = nbbadobsmono;

    if (state_->debug_)
    {
        std::cout << "\n \t>>> localBA() --> Nb of bad obs / nb removed map point : " << nbbadobs << " / " << nbbadlm;
        std::cout << "\n \t>>> localBA() --> Nb of bad obs mono : " << nbbadobsmono;
    }
}
