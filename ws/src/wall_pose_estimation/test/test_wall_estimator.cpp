#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "wall_pose_estimation/wall_estimator.hpp"

using wall_pose_estimation::EstimatorConfig;
using wall_pose_estimation::Point2D;
using wall_pose_estimation::WallEstimator;

namespace {

constexpr double kDeg = M_PI / 180.0;

/// Builds a noise-free scan of an infinite wall at perpendicular distance d with normal angle psi.
std::vector<float> makeWallScan(double d, double psi, double angle_min, double angle_inc, int n) {
    std::vector<float> ranges(n, std::numeric_limits<float>::infinity());
    for (int i = 0; i < n; ++i) {
        double theta = angle_min + i * angle_inc;
        double c = std::cos(theta - psi);
        if (c > 1e-3) {
            ranges[i] = static_cast<float>(d / c);
        }
    }
    return ranges;
}

}  // namespace

TEST(ExtractFrontPoints, IgnoresInvalidReturns) {
    WallEstimator est;
    std::vector<float> ranges = {1.0f, std::nanf(""), std::numeric_limits<float>::infinity(),
                                 0.01f, 50.0f};
    // 5 rays from -1 deg to +1 deg, all inside the front field of view
    auto points = est.extractFrontPoints(ranges, -1.0 * kDeg, 0.5 * kDeg, 0.1, 10.0);
    EXPECT_EQ(points.size(), 1u);
}

TEST(FitLine, WallStraightAhead) {
    std::vector<Point2D> points;
    for (int i = -10; i <= 10; ++i) {
        points.push_back({2.0, 0.1 * i});  // vertical line x = 2
    }
    auto result = WallEstimator::fitLine(points);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->distance, 2, 1e-6);
    EXPECT_NEAR(result->angle, 0.0, 1e-6);
}

TEST(Estimate, RotatedWall) {
    WallEstimator est;
    const double angle_min = -135.0 * kDeg;
    const double angle_inc = 0.5 * kDeg;
    auto ranges = makeWallScan(1.5, 15.0 * kDeg, angle_min, angle_inc, 541);
    auto result = est.estimate(ranges, angle_min, angle_inc, 0.1, 10.0);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->distance, 1.5, 1e-3);
    EXPECT_NEAR(result->angle, 15.0 * kDeg, 1e-3);
}
