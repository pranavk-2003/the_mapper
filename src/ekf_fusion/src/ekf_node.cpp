#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Dense>
#include <cmath>

class EKFNode : public rclcpp::Node {
public:
    EKFNode() : Node("ekf_node") {
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu_plugin/out", 10, std::bind(&EKFNode::imuCallback, this, std::placeholders::_1));
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&EKFNode::lidarCallback, this, std::placeholders::_1));
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_ekf", 10);

        state_ = Eigen::VectorXd::Zero(5);  // State: [x, y, vx, vy, yaw]
        covariance_ = Eigen::MatrixXd::Identity(5, 5);  // Covariance matrix
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    Eigen::VectorXd state_;
    Eigen::MatrixXd covariance_;
    double dt_ = 0.1;  // Time step
    double process_noise_ = 0.1;
    double measurement_noise_ = 0.01;

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        double yaw_rate = msg->angular_velocity.z;
        double ax = msg->linear_acceleration.x;
        double ay = msg->linear_acceleration.y;

        predict(ax, ay, yaw_rate, dt_);
    }

    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        double min_range = std::numeric_limits<double>::infinity();
        double lidar_x = 0.0;
        double lidar_y = 0.0;

        for (size_t i = 0; i < msg->ranges.size(); i++) {
            if (msg->ranges[i] >= msg->range_min && msg->ranges[i] <= msg->range_max) {
                if (msg->ranges[i] < min_range) {
                    min_range = msg->ranges[i];
                    double angle = msg->angle_min + i * msg->angle_increment;
                    lidar_x = min_range * std::cos(angle);
                    lidar_y = min_range * std::sin(angle);
                }
            }
        }

        Eigen::Vector2d measurement(lidar_x, lidar_y);
        update(measurement);
    }

    void predict(double ax, double ay, double yaw_rate, double dt) {
        state_(2) += ax * dt;  // vx
        state_(3) += ay * dt;  // vy
        state_(0) += state_(2) * dt;  // x
        state_(1) += state_(3) * dt;  // y
        state_(4) += yaw_rate * dt;  // yaw

        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(5, 5);
        F(0, 2) = dt;
        F(1, 3) = dt;

        covariance_ = F * covariance_ * F.transpose() + process_noise_ * Eigen::MatrixXd::Identity(5, 5);
    }

    void update(const Eigen::Vector2d& measurement) {
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 5);
        H(0, 0) = 1;
        H(1, 1) = 1;

        Eigen::VectorXd predicted_measurement = H * state_;
        Eigen::Vector2d innovation = measurement - predicted_measurement;

        Eigen::MatrixXd S = H * covariance_ * H.transpose() + measurement_noise_ * Eigen::MatrixXd::Identity(2, 2);
        Eigen::MatrixXd K = covariance_ * H.transpose() * S.inverse();

        state_ += K * innovation;
        covariance_ = (Eigen::MatrixXd::Identity(5, 5) - K * H) * covariance_;

        publishOdometry();
    }

    void publishOdometry() {
        auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
        odom_msg->header.stamp = this->now();
        odom_msg->header.frame_id = "odom";
        odom_msg->child_frame_id = "base_link";
        
        odom_msg->pose.pose.position.x = state_(0);
        odom_msg->pose.pose.position.y = state_(1);
        odom_msg->pose.pose.position.z = 0.0;

        double yaw = state_(4);
        odom_msg->pose.pose.orientation.x = 0.0;
        odom_msg->pose.pose.orientation.y = 0.0;
        odom_msg->pose.pose.orientation.z = std::sin(yaw / 2.0);
        odom_msg->pose.pose.orientation.w = std::cos(yaw / 2.0);

        odom_msg->twist.twist.linear.x = state_(2);
        odom_msg->twist.twist.linear.y = state_(3);
        odom_msg->twist.twist.angular.z = yaw;

        odom_pub_->publish(*odom_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto ekf_node = std::make_shared<EKFNode>();
    rclcpp::spin(ekf_node);
    rclcpp::shutdown();
    return 0;
}