#include "wall_pose_estimation/wall_estimator.hpp"

#include <cmath>

namespace wall_pose_estimation {

WallEstimator::WallEstimator(const EstimatorConfig& config) : config_(config) {}

std::vector<Point2D> WallEstimator::extractFrontPoints(const std::vector<float>& ranges,
                                                       double angle_min,
                                                       double angle_increment,
                                                       double range_min,
                                                       double range_max) const {
    std::vector<Point2D> points;

    float angle = angle_min;

    for(float range : ranges){
        if (std::abs(angle) <= config_.fov /2 && range_min<=range && range<= range_max){
            Point2D point = {range * cos(angle), range * sin(angle)};
            points.push_back(point);
        }
        angle += angle_increment;
    }


    return points;
}

std::optional<WallEstimate> WallEstimator::fitLine(const std::vector<Point2D>& points) {

    if (points.size() <= 2){ 
        return std::nullopt;
    }

    // centroids
    float mx = 0, my = 0;
    for (Point2D point: points){
        mx += point.x;
        my += point.y;
    }
    mx /= points.size();
    my /= points.size();

    // covariance, not normalized since it cancels out later
    float sxx = 0, syy = 0, sxy = 0;
    for (Point2D point: points){
        sxx += std::pow((point.x - mx), 2);
        syy += std::pow((point.y - my), 2);
        sxy += (point.x - mx) * (point.y - my);
    }

    // all points in the same spot (should not happen)
    float ZERO = 0.000001;
    if (std::abs(sxx) <= ZERO && std::abs(syy) <= ZERO){
        return std::nullopt;
    }

    
    float phi =  0.5 * atan2(2*sxy, sxx - syy); // line direction
    float a = phi + M_PI/2;                     // normal angle
    float d = cos(a)*mx + sin(a)*my;            //distance to line

    // fix ranges
    if (d<0){
        a += M_PI;
        d = -d;
    }
    a = remainderf(a, 2* M_PI);

    // rms
    float rms_residual = 0;
    for (Point2D point: points){
        rms_residual += std::pow((cos(a)*point.x + sin(a)*point.y - d), 2);
    }
    rms_residual /= points.size();
    rms_residual = sqrt(rms_residual);
    

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
    long unsigned int i = 0;
    while (i < points.size()){
        Point2D point = points[i];
        float distancePointToLine = std::abs(point.x * cos(wallEstimate.value().angle) + 
                                  point.y * sin(wallEstimate.value().angle) - wallEstimate.value().distance);

        if (distancePointToLine > config_.inlier_threshold){
            points.erase (points.begin() + i);
        }
        else{
            i += 1;
        }
    }

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
