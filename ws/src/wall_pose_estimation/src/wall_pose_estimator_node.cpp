#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

#include "wall_pose_estimation/wall_estimator.hpp"

namespace wall_pose_estimation {

/// ROS 2 wrapper around WallEstimator.
///
/// Subscribes:  /scan            (sensor_msgs/msg/LaserScan)
/// Publishes:   /wall/pose       (geometry_msgs/msg/PoseStamped)  foot point of the perpendicular on the
///                                                                wall, oriented along the wall normal
///              /wall/distance   (std_msgs/msg/Float64)           perpendicular distance [m]
class WallPoseEstimatorNode : public rclcpp::Node {
public:
    WallPoseEstimatorNode() : Node("wall_pose_estimator") {

        // parameters, set by params.yaml, fallback default EstimatorConfig values
        EstimatorConfig config;
        config.fov = this->declare_parameter("fov", config.fov);
        config.min_points = static_cast<std::size_t>(
            this->declare_parameter("min_points", static_cast<int64_t>(config.min_points)));
        config.inlier_threshold = this->declare_parameter("inlier_threshold", config.inlier_threshold);
        config.max_rms_residual = this->declare_parameter("max_rms_residual", config.max_rms_residual);
        estimator_ = WallEstimator(config);
        wallPosePublisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("wall/pose", 10);
        wallDistPublisher_ = this->create_publisher<std_msgs::msg::Float64>("wall/distance", 10);

        laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(), std::bind(&WallPoseEstimatorNode::onScan, this, std::placeholders::_1)); //SensorDataQoS(), 10 was reliable the queue, now best effort

    }

private:
    void onScan(const sensor_msgs::msg::LaserScan & msg) {
        std::optional<WallEstimate> wallEstimate = estimator_.estimate(msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max);

        if (wallEstimate.has_value()){

            // fill distance msg
            auto distMessage = std_msgs::msg::Float64();
            distMessage.data = wallEstimate.value().distance;

            // fill pose msg
            auto poseMessage = geometry_msgs::msg::PoseStamped();
            poseMessage.header = msg.header; 
            poseMessage.pose.position.x = wallEstimate.value().distance *
                                          std::cos(wallEstimate.value().angle);
            poseMessage.pose.position.y = wallEstimate.value().distance *
                                          std::sin(wallEstimate.value().angle);
            poseMessage.pose.orientation.z = std::sin(wallEstimate.value().angle / 2.0);
            poseMessage.pose.orientation.w = std::cos(wallEstimate.value().angle / 2.0); 

            // publish both
             wallPosePublisher_-> publish(poseMessage);
             wallDistPublisher_-> publish(distMessage);
        }
        else{
             //does not flood the terminal if it repeats a lotunlike info
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No reliable wall estimate");
        }
        
    }

    WallEstimator estimator_; 

    // subscription and publisher members
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_; 
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr wallPosePublisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr wallDistPublisher_;
    
};


}  // namespace wall_pose_estimation

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<wall_pose_estimation::WallPoseEstimatorNode>());
    rclcpp::shutdown();
    return 0;
}
