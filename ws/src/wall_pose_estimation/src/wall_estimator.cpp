#include "wall_pose_estimation/wall_estimator.hpp"

#include <algorithm>
#include <cmath>

namespace wall_pose_estimation {

WallEstimator::WallEstimator(const EstimatorConfig& config) : config_(config) {}

std::vector<Point2D> WallEstimator::extractFrontPoints(const std::vector<float>& ranges,
                                                       double angle_min,
                                                       double angle_increment,
                                                       double range_min,
                                                       double range_max) const {
    std::vector<Point2D> points;

    for (std::size_t i = 0; i < ranges.size(); ++i) {
        
        const double angle = angle_min + static_cast<double>(i) * angle_increment;
        const double range = ranges[i];
        if (std::abs(angle) <= config_.fov / 2 && range_min <= range && range <= range_max) {
            points.push_back({range * std::cos(angle), range * std::sin(angle)});
        }
    }

    return points;
}

std::optional<WallEstimate> WallEstimator::fitLine(const std::vector<Point2D>& points) {

    if (points.size() < 2){
        return std::nullopt;
    }

    // centroids
    double mx = 0, my = 0;
    for (Point2D point: points){
        mx += point.x;
        my += point.y;
    }
    mx /= points.size();
    my /= points.size();

    // covariance, not normalized since it cancels out later
    double sxx = 0, syy = 0, sxy = 0;
    for (Point2D point: points){
        const double dx = point.x - mx;
        const double dy = point.y - my;
        sxx += dx * dx;
        syy += dy * dy;
        sxy += dx * dy;
    }

    // all points in the same spot (should not happen)
    constexpr double kEpsilon = 1e-6;
    if (std::abs(sxx) <= kEpsilon && std::abs(syy) <= kEpsilon){
        return std::nullopt;
    }

    const double phi = 0.5 * std::atan2(2 * sxy, sxx - syy);  // line direction
    double a = phi + M_PI / 2;                                  // normal angle
    double d = std::cos(a) * mx + std::sin(a) * my;             // distance to line

    // fix ranges
    if (d<0){
        a += M_PI;
        d = -d;
    }
    a = std::remainder(a, 2 * M_PI);

    // rms
    const double cos_a = std::cos(a), sin_a = std::sin(a);
    double sum_sq = 0;
    for (Point2D point: points){
        const double r = cos_a * point.x + sin_a * point.y - d;
        sum_sq += r * r;
    }
    const double rms_residual = std::sqrt(sum_sq / points.size());

    WallEstimate wallEstimate = {d, a, rms_residual, points.size()};
    return wallEstimate;
}

std::optional<WallEstimate> WallEstimator::estimate(const std::vector<float>& ranges,
                                                    double angle_min,
                                                    double angle_increment,
                                                    double range_min,
                                                    double range_max) const {
    // extract -> fit -> keep inliers -> refit -> validate

    std::vector<Point2D> points = extractFrontPoints(ranges, angle_min, angle_increment, range_min, range_max);
                            
    if (points.size() < config_.min_points){
        return std::nullopt;
    }

    std::optional<WallEstimate> wallEstimate = fitLine(points);
    if (wallEstimate == std::nullopt){
        return std::nullopt;
    }

    // remove outliers
    const double cos_a = std::cos(wallEstimate->angle);
    const double sin_a = std::sin(wallEstimate->angle);
    const double d = wallEstimate->distance;
    points.erase(std::remove_if(points.begin(), points.end(),
                                [&](const Point2D& point) {
                                    return std::abs(point.x * cos_a + point.y * sin_a - d) >
                                           config_.inlier_threshold;
                                }),
                 points.end());

    // re-estimate
    wallEstimate = fitLine(points);
    if (wallEstimate == std::nullopt){
        return std::nullopt;
    }

    if (wallEstimate.value().rms_residual > config_.max_rms_residual){
        return std::nullopt;
    }
    return wallEstimate;
}

}  // namespace wall_pose_estimation
