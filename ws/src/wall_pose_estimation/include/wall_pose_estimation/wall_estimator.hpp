#pragma once

#include <cstddef>
#include <optional>
#include <vector>

namespace wall_pose_estimation {

/// A 2D point in the LiDAR frame (x forward, y left), in meters.
struct Point2D {
    double x;
    double y;
};

/// Result of fitting a wall (a straight line) in front of the drone.
struct WallEstimate {
    double distance;           ///< Perpendicular distance from the sensor origin to the wall [m]
    double angle;              ///< Angle of the wall normal in the sensor frame [rad].
                               ///< 0 = the drone faces the wall head-on, positive = wall normal to the left.
    double rms_residual;       ///< Root-mean-square distance of the inlier points to the fitted line [m]
    std::size_t num_points;    ///< Number of points used in the final fit
};

/// Tuning parameters. Defaults are reasonable for the provided bag.
struct EstimatorConfig {
    double fov = 1.0472;              ///< Total front field of view used for the fit [rad] (60 deg, i.e. +/-30 deg)
    std::size_t min_points = 20;      ///< Minimum number of points required for a valid estimate
    double inlier_threshold = 0.05;   ///< Max point-to-line distance to keep a point when refitting [m]
    double max_rms_residual = 0.03;   ///< Reject the estimate if the fit is worse than this [m]
};

/// Estimates distance and relative orientation of a flat surface (wall) in front of the drone
/// from a single 2D LiDAR scan. Used to align the EMAT probe perpendicular to the surface.
///
/// This class has no ROS dependency on purpose, so it can be unit tested in isolation.
class WallEstimator {
public:
    explicit WallEstimator(const EstimatorConfig& config = EstimatorConfig{});

    /// Step 1: Converts the valid returns inside the front field of view to Cartesian points.
    /// A return is valid if it is finite and within [range_min, range_max].
    /// The ray with index i has angle  angle_min + i * angle_increment.
    std::vector<Point2D> extractFrontPoints(const std::vector<float>& ranges,
                                            double angle_min,
                                            double angle_increment,
                                            double range_min,
                                            double range_max) const;

    /// Step 2: Fits a line to the points (total least squares) and returns distance and
    /// normal angle of that line relative to the origin. Returns std::nullopt if fewer than
    /// 2 points are given or the fit is degenerate.
    static std::optional<WallEstimate> fitLine(const std::vector<Point2D>& points);

    /// Step 3: Full pipeline: extract points, fit, reject outliers, refit, validate.
    /// Returns std::nullopt if no reliable wall is found.
    std::optional<WallEstimate> estimate(const std::vector<float>& ranges,
                                         double angle_min,
                                         double angle_increment,
                                         double range_min,
                                         double range_max) const;

    const EstimatorConfig& config() const { return config_; }

private:
    EstimatorConfig config_;
};

}  // namespace wall_pose_estimation
