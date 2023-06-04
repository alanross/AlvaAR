#include "frame.hpp"

Frame::Frame()
        : id_(-1), keyframeId_(0), timestamp_(0.), numKeypoints_(0), numKeypoints2d_(0), numKeypoints3d_(0),
          Frl_(Eigen::Matrix3d::Zero()), Fcv_(cv::Mat::zeros(3, 3, CV_64F))
{}


Frame::Frame(std::shared_ptr<CameraCalibration> cameraCalibration, const size_t cellSize)
        : id_(-1), keyframeId_(0), timestamp_(0.), cellSize_(cellSize), numKeypoints_(0),
          numKeypoints2d_(0), numKeypoints3d_(0), cameraCalibration_(cameraCalibration)
{
    // Init grid from images size
    numCellsW_ = static_cast<size_t>(ceilf(static_cast<float>(cameraCalibration_->imgWidth_) / cellSize_));
    numCellsH_ = static_cast<size_t>(ceilf(static_cast<float>(cameraCalibration_->imgHeight_) / cellSize_));
    gridCells_ = numCellsW_ * numCellsH_;
    numOccupiedCells_ = 0;

    gridKeypointsIds_.resize(gridCells_);
}

Frame::Frame(const Frame &frame)
        : id_(frame.id_), keyframeId_(frame.keyframeId_), timestamp_(frame.timestamp_), mapKeypoints_(frame.mapKeypoints_), gridKeypointsIds_(frame.gridKeypointsIds_), gridCells_(frame.gridCells_), numOccupiedCells_(frame.numOccupiedCells_),
          cellSize_(frame.cellSize_), numCellsW_(frame.numCellsW_), numCellsH_(frame.numCellsH_), numKeypoints_(frame.numKeypoints_), numKeypoints2d_(frame.numKeypoints2d_), numKeypoints3d_(frame.numKeypoints3d_),
          Twc_(frame.Twc_), Tcw_(frame.Tcw_), cameraCalibration_(frame.cameraCalibration_),
          Frl_(frame.Frl_), Fcv_(frame.Fcv_), covisibleKeyframeIds_(frame.covisibleKeyframeIds_), localMapPointIds_(frame.localMapPointIds_)
{}

std::vector<Keypoint> Frame::getKeypoints() const
{
    std::vector<Keypoint> v;
    v.reserve(numKeypoints_);
    for (const auto &kp: mapKeypoints_)
    {
        v.push_back(kp.second);
    }
    return v;
}

std::vector<Keypoint> Frame::getKeypoints2d() const
{
    std::vector<Keypoint> v;
    v.reserve(numKeypoints2d_);
    for (const auto &kp: mapKeypoints_)
    {
        if (!kp.second.is3d_)
        {
            v.push_back(kp.second);
        }
    }
    return v;
}

std::vector<Keypoint> Frame::getKeypoints3d() const
{
    std::vector<Keypoint> result;
    result.reserve(numKeypoints3d_);
    for (const auto &kp: mapKeypoints_)
    {
        if (kp.second.is3d_)
        {
            result.push_back(kp.second);
        }
    }
    return result;
}

std::vector<cv::Point2f> Frame::getKeypointsPx() const
{
    std::vector<cv::Point2f> result;
    result.reserve(numKeypoints_);
    for (const auto &kp: mapKeypoints_)
    {
        result.push_back(kp.second.px_);
    }
    return result;
}

std::vector<cv::Point2f> Frame::getKeypointsUnPx() const
{
    std::vector<cv::Point2f> result;
    result.reserve(numKeypoints_);
    for (const auto &kp: mapKeypoints_)
    {
        result.push_back(kp.second.unpx_);
    }
    return result;
}

Keypoint Frame::getKeypointById(const int keypointId) const
{
    auto it = mapKeypoints_.find(keypointId);
    if (it == mapKeypoints_.end())
    {
        return Keypoint();
    }

    return it->second;
}

inline void Frame::computeKeypoint(const cv::Point2f &point, Keypoint &keypoint)
{
    keypoint.px_ = point;
    keypoint.unpx_ = cameraCalibration_->undistortImagePoint(point);

    Eigen::Vector3d unPx(keypoint.unpx_.x, keypoint.unpx_.y, 1.);
    keypoint.bv_ = cameraCalibration_->inverseK_ * unPx;
    keypoint.bv_.normalize();
}

inline Keypoint Frame::computeKeypoint(const cv::Point2f &point, const int keypointId)
{
    Keypoint keypoint;
    keypoint.keypointId_ = keypointId;
    computeKeypoint(point, keypoint);
    return keypoint;
}

void Frame::addKeypoint(const Keypoint &keypoint)
{
    if (mapKeypoints_.count(keypoint.keypointId_))
    {
        return;
    }

    mapKeypoints_.emplace(keypoint.keypointId_, keypoint);
    addKeypointToGrid(keypoint);

    numKeypoints_++;

    if (keypoint.is3d_)
    {
        numKeypoints3d_++;
    }
    else
    {
        numKeypoints2d_++;
    }
}

void Frame::addKeypoint(const cv::Point2f &pt, const int keypointId)
{
    Keypoint keypoint = computeKeypoint(pt, keypointId);

    addKeypoint(keypoint);
}

void Frame::addKeypoint(const cv::Point2f &point, const int keypointId, const cv::Mat &descriptor)
{
    Keypoint keypoint = computeKeypoint(point, keypointId);
    keypoint.desc_ = descriptor;

    addKeypoint(keypoint);
}

void Frame::updateKeypoint(const int keypointId, const cv::Point2f &point)
{
    auto it = mapKeypoints_.find(keypointId);
    if (it == mapKeypoints_.end())
    {
        return;
    }

    Keypoint keypoint = it->second;

    computeKeypoint(point, keypoint);

    updateKeypointInGrid(it->second, keypoint);
    it->second = keypoint;
}

void Frame::updateKeypointDesc(const int keypointId, const cv::Mat &descriptor)
{
    auto it = mapKeypoints_.find(keypointId);
    if (it == mapKeypoints_.end())
    {
        return;
    }

    it->second.desc_ = descriptor;
}

bool Frame::updateKeypointId(const int prevKeypointId, const int newKeypointId, const bool is3d)
{
    if (mapKeypoints_.count(newKeypointId))
    {
        return false;
    }

    auto it = mapKeypoints_.find(prevKeypointId);
    if (it == mapKeypoints_.end())
    {
        return false;
    }

    Keypoint keypoint = it->second;
    keypoint.keypointId_ = newKeypointId;
    keypoint.is3d_ = is3d;
    removeKeypointById(prevKeypointId);
    addKeypoint(keypoint);

    return true;
}

void Frame::removeKeypointById(const int keypointId)
{
    auto it = mapKeypoints_.find(keypointId);
    if (it == mapKeypoints_.end())
    {
        return;
    }

    removeKeypointFromGrid(it->second);

    if (it->second.is3d_)
    {
        numKeypoints3d_--;
    }
    else
    {
        numKeypoints2d_--;
    }

    numKeypoints_--;

    mapKeypoints_.erase(keypointId);
}

void Frame::turnKeypoint3d(const int keypointId)
{
    auto it = mapKeypoints_.find(keypointId);
    if (it == mapKeypoints_.end())
    {
        return;
    }

    if (!it->second.is3d_)
    {
        it->second.is3d_ = true;
        numKeypoints3d_++;
        numKeypoints2d_--;
    }
}

bool Frame::isObservingKeypoint(const int keypointId) const
{
    return mapKeypoints_.count(keypointId);
}

void Frame::addKeypointToGrid(const Keypoint &keypoint)
{
    int idx = getKeypointCellIdx(keypoint.px_);

    if (gridKeypointsIds_.at(idx).empty())
    {
        numOccupiedCells_++;
    }

    gridKeypointsIds_.at(idx).push_back(keypoint.keypointId_);
}

void Frame::removeKeypointFromGrid(const Keypoint &keypoint)
{
    int index = getKeypointCellIdx(keypoint.px_);

    if (index < 0 || index >= (int) gridKeypointsIds_.size())
    {
        return;
    }

    size_t n = gridKeypointsIds_.at(index).size();

    for (size_t i = 0; i < n; i++)
    {
        if (gridKeypointsIds_.at(index).at(i) == keypoint.keypointId_)
        {
            gridKeypointsIds_.at(index).erase(gridKeypointsIds_.at(index).begin() + i);

            if (gridKeypointsIds_.at(index).empty())
            {
                numOccupiedCells_--;
            }
            break;
        }
    }
}

void Frame::updateKeypointInGrid(const Keypoint &prevKeypoint, const Keypoint &newKeypoint)
{
    int prevIndex = getKeypointCellIdx(prevKeypoint.px_);
    int newIndex = getKeypointCellIdx(newKeypoint.px_);

    if (prevIndex == newIndex)
    {
        return;
    }
    else
    {
        // First remove kp
        removeKeypointFromGrid(prevKeypoint);
        // Second the new kp is added to the grid
        addKeypointToGrid(newKeypoint);
    }
}

int Frame::getKeypointCellIdx(const cv::Point2f &point) const
{
    int r = floor(point.y / cellSize_);
    int c = floor(point.x / cellSize_);
    return (r * numCellsW_ + c);
}

std::vector<Keypoint> Frame::getSurroundingKeypoints(const cv::Point2f &point) const
{
    std::vector<Keypoint> keypoints;
    keypoints.reserve(20);

    int rkp = floor(point.y / cellSize_);
    int ckp = floor(point.x / cellSize_);

    for (int r = rkp - 1; r < rkp + 1; r++)
    {
        for (int c = ckp - 1; c < ckp + 1; c++)
        {
            int idx = r * numCellsW_ + c;
            if (r < 0 || c < 0 || idx > (int) gridKeypointsIds_.size())
            {
                continue;
            }
            for (const auto &id: gridKeypointsIds_.at(idx))
            {
                auto it = mapKeypoints_.find(id);
                if (it != mapKeypoints_.end())
                {
                    keypoints.push_back(it->second);
                }
            }
        }
    }
    return keypoints;
}

std::map<int, int> Frame::getCovisibleKeyframeMap() const
{
    return covisibleKeyframeIds_;
}

void Frame::addCovisibleKeyframe(const int keyframeId)
{
    if (keyframeId == keyframeId_)
    {
        return;
    }

    auto it = covisibleKeyframeIds_.find(keyframeId);
    if (it != covisibleKeyframeIds_.end())
    {
        it->second += 1;
    }
    else
    {
        covisibleKeyframeIds_.emplace(keyframeId, 1);
    }
}

void Frame::removeCovisibleKeyframe(const int keyframeId)
{
    if (keyframeId == keyframeId_)
    {
        return;
    }

    covisibleKeyframeIds_.erase(keyframeId);
}

void Frame::decreaseCovisibleKeyframe(const int keyframeId)
{
    if (keyframeId == keyframeId_)
    {
        return;
    }

    auto it = covisibleKeyframeIds_.find(keyframeId);
    if (it != covisibleKeyframeIds_.end())
    {
        if (it->second != 0)
        {
            it->second -= 1;
            if (it->second == 0)
            {
                covisibleKeyframeIds_.erase(it);
            }
        }
    }
}

Sophus::SE3d Frame::getTcw() const
{
    return Tcw_;
}

Sophus::SE3d Frame::getTwc() const
{
    return Twc_;
}

Eigen::Matrix3d Frame::getRcw() const
{
    return Tcw_.rotationMatrix();
}

Eigen::Matrix3d Frame::getRwc() const
{
    return Twc_.rotationMatrix();
}

void Frame::setTwc(const Sophus::SE3d &Twc)
{
    Twc_ = Twc;
    Tcw_ = Twc.inverse();
}

void Frame::setTwc(const Eigen::Matrix3d &Rwc, Eigen::Vector3d &twc)
{
    Twc_.setRotationMatrix(Rwc);
    Twc_.translation() = twc;
    Tcw_ = Twc_.inverse();
}

cv::Point2f Frame::projCamToImage(const Eigen::Vector3d &point) const
{
    return cameraCalibration_->projectCamToImage(point);
}

cv::Point2f Frame::projCamToImageDist(const Eigen::Vector3d &point) const
{
    return cameraCalibration_->projectCamToImageDist(point);
}

Eigen::Vector3d Frame::projCamToWorld(const Eigen::Vector3d &point) const
{
    Eigen::Vector3d wpt = Twc_ * point;

    return wpt;
}

Eigen::Vector3d Frame::projWorldToCam(const Eigen::Vector3d &point) const
{
    Eigen::Vector3d camPoint = Tcw_ * point;

    return camPoint;
}

cv::Point2f Frame::projWorldToImage(const Eigen::Vector3d &point) const
{
    return cameraCalibration_->projectCamToImage(projWorldToCam(point));
}

cv::Point2f Frame::projWorldToImageDist(const Eigen::Vector3d &point) const
{
    return cameraCalibration_->projectCamToImageDist(projWorldToCam(point));
}

bool Frame::isInImage(const cv::Point2f &point) const
{
    return (point.x >= 0 && point.y >= 0 && point.x < cameraCalibration_->imgWidth_ && point.y < cameraCalibration_->imgHeight_);
}

void Frame::reset()
{
    id_ = -1;
    keyframeId_ = 0;
    timestamp_ = 0.;

    mapKeypoints_.clear();
    gridKeypointsIds_.clear();
    gridKeypointsIds_.resize(gridCells_);

    numKeypoints_ = 0;
    numKeypoints2d_ = 0;
    numKeypoints3d_ = 0;

    numOccupiedCells_ = 0;

    Twc_ = Sophus::SE3d();
    Tcw_ = Sophus::SE3d();

    covisibleKeyframeIds_.clear();
    localMapPointIds_.clear();
}