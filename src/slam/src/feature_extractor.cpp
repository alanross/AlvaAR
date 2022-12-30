#include "feature_extractor.hpp"
#include <algorithm>
#include <iterator>

cv::Ptr<cv::DescriptorExtractor> descriptor_;

FeatureExtractor::FeatureExtractor(double maxQuality) : maxQuality_(maxQuality)
{
}

std::vector<cv::Point2f> FeatureExtractor::detectFeaturePoints(const cv::Mat &image, const int cellSize, const std::vector<cv::Point2f> &currKeypoints, const cv::Rect &roi)
{
    if (image.empty())
    {
        return std::vector<cv::Point2f>();
    }

    size_t numCols = image.cols;
    size_t numRows = image.rows;
    size_t cellSizeHalf = cellSize / 4;
    size_t numCellsH = numRows / cellSize;
    size_t numCellsW = numCols / cellSize;
    size_t numCells = numCellsH * numCellsW;

    std::vector<cv::Point2f> detectedPx;
    detectedPx.reserve(numCells);

    std::vector<std::vector<bool>> occupiedCells(numCellsH + 1, std::vector<bool>(numCellsW + 1, false));

    cv::Mat mask = cv::Mat::ones(image.rows, image.cols, CV_32F);

    for (const auto &px: currKeypoints)
    {
        occupiedCells[px.y / cellSize][px.x / cellSize] = true;
        cv::circle(mask, px, cellSizeHalf, cv::Scalar(0.), -1);
    }

    size_t numOccupied = 0;

    std::vector<std::vector<cv::Point2f>> matrixDetectedPx(numCells);
    std::vector<std::vector<cv::Point2f>> matrixSecDetectionsPx(numCells);

    auto cvRange = cv::Range(0, numCells);

    parallel_for_(cvRange, [&](const cv::Range &range) {
        for (int i = range.start; i < range.end; i++)
        {
            size_t r = floor(i / numCellsW);
            size_t c = i % numCellsW;

            if (occupiedCells[r][c])
            {
                numOccupied++;
                continue;
            }

            size_t x = c * cellSize;
            size_t y = r * cellSize;

            cv::Rect regionOfInterest(x, y, cellSize, cellSize);

            if (x + cellSize < numCols - 1 && y + cellSize < numRows - 1)
            {
                cv::Mat hMap;
                cv::Mat filteredImage;
                cv::GaussianBlur(image(regionOfInterest), filteredImage, cv::Size(3, 3), 0.);
                cv::cornerMinEigenVal(filteredImage, hMap, 3, 3);

                double minVal;
                double maxVal;
                cv::Point minPx;
                cv::Point maxPx;

                cv::minMaxLoc(hMap.mul(mask(regionOfInterest)), &minVal, &maxVal, &minPx, &maxPx);
                maxPx.x += x;
                maxPx.y += y;

                if (maxPx.x < roi.x || maxPx.y < roi.y || maxPx.x >= roi.x + roi.width || maxPx.y >= roi.y + roi.height)
                {
                    continue;
                }

                if (maxVal >= maxQuality_)
                {
                    matrixDetectedPx.at(i).push_back(maxPx);
                    cv::circle(mask, maxPx, cellSizeHalf, cv::Scalar(0.), -1);
                }

                cv::minMaxLoc(hMap.mul(mask(regionOfInterest)), &minVal, &maxVal, &minPx, &maxPx);
                maxPx.x += x;
                maxPx.y += y;

                if (maxPx.x < roi.x || maxPx.y < roi.y || maxPx.x >= roi.x + roi.width || maxPx.y >= roi.y + roi.height)
                {
                    continue;
                }

                if (maxVal >= maxQuality_)
                {
                    matrixSecDetectionsPx.at(i).push_back(maxPx);
                    cv::circle(mask, maxPx, cellSizeHalf, cv::Scalar(0.), -1);
                }
            }
        }
    });

    for (const auto &px: matrixDetectedPx)
    {
        if (!px.empty())
        {
            detectedPx.insert(detectedPx.end(), px.begin(), px.end());
        }
    }

    size_t numKeypoints = detectedPx.size();

    if (numKeypoints + numOccupied < numCells)
    {
        size_t numSec = numCells - (numKeypoints + numOccupied);
        size_t k = 0;

        for (const auto &secKeypoints: matrixSecDetectionsPx)
        {
            if (!secKeypoints.empty())
            {
                detectedPx.push_back(secKeypoints.back());
                k++;
                if (k == numSec)
                {
                    break;
                }
            }
        }
    }

    numKeypoints = detectedPx.size();

    if (numKeypoints < 0.33 * (numCells - numOccupied))
    {
        maxQuality_ *= 0.5;
    }
    else if (numKeypoints > 0.9 * (numCells - numOccupied))
    {
        maxQuality_ *= 1.5;
    }

    // Compute Corners with Sub-Pixel Accuracy
    if (!detectedPx.empty())
    {
        // Set the need parameters to find the refined corners
        cv::Size winSize = cv::Size(3, 3);
        cv::Size zeroZone = cv::Size(-1, -1);
        cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.01);
        cv::cornerSubPix(image, detectedPx, winSize, zeroZone, criteria);
    }

    return detectedPx;
}

std::vector<cv::Mat> FeatureExtractor::describeFeaturePoints(const cv::Mat &image, const std::vector<cv::Point2f> &points) const
{
    if (points.empty())
    {
        return std::vector<cv::Mat>();
    }

    std::vector<cv::KeyPoint> keypoints;
    size_t numKeypoints = points.size();
    keypoints.reserve(numKeypoints);
    std::vector<cv::Mat> descriptors;
    descriptors.reserve(numKeypoints);

    cv::KeyPoint::convert(points, keypoints);

    cv::Mat descs;

    if (descriptor_ == nullptr)
    {
        descriptor_ = cv::ORB::create(500, 1., 0);
    }

    descriptor_->compute(image, keypoints, descs);

    if (keypoints.empty())
    {
        return std::vector<cv::Mat>(numKeypoints, cv::Mat());
    }

    size_t k = 0;

    for (size_t i = 0; i < numKeypoints; i++)
    {
        if (k < keypoints.size())
        {
            if (keypoints[k].pt == points[i])
            {
                descriptors.push_back(descs.row(k));
                k++;
            }
            else
            {
                descriptors.push_back(cv::Mat());
            }
        }
        else
        {
            descriptors.push_back(cv::Mat());
        }
    }

    assert(descriptors.size() == points.size());

    return descriptors;
}
