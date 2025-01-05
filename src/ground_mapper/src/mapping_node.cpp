#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

class MappingNode : public rclcpp::Node
{
public:
    MappingNode() : Node("mapping_node"), pitch_(0.0), roll_(0.0)
    {
        // Subscribe to laser scan and IMU data
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&MappingNode::laserCallback, this, std::placeholders::_1));
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu_plugin/out", 10, std::bind(&MappingNode::imuCallback, this, std::placeholders::_1));

        // Publisher for point cloud
        point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/assembled_cloud", 10);
    }

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Create a PointCloud2 message
        auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        cloud_msg->header = msg->header;
        cloud_msg->height = 1;
        cloud_msg->width = msg->ranges.size();
        cloud_msg->is_dense = false;
        cloud_msg->is_bigendian = false;

        // Set up the point cloud fields (x, y, z)
        sensor_msgs::PointCloud2Modifier modifier(*cloud_msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(msg->ranges.size());

        // Iterate over the laser scan data and convert each point to 3D
        sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");

        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            float range = msg->ranges[i];
            if (std::isfinite(range))
            {
                // Calculate the angle of the laser point in 2D plane
                float angle = msg->angle_min + i * msg->angle_increment;

                // Convert 2D scan point to 3D using pitch and roll from IMU
                float x = range * cos(angle);
                float y = range * sin(angle);
                float z = 0.0;

                // Apply rotation based on IMU pitch and roll
                float rotated_x = x;
                float rotated_y = y * cos(roll_) - z * sin(roll_);
                float rotated_z = y * sin(roll_) + z * cos(roll_);

                // Store the rotated coordinates
                *iter_x = rotated_x;
                *iter_y = rotated_y;
                *iter_z = rotated_z;
            }
            else
            {
                *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
            }
            ++iter_x;
            ++iter_y;
            ++iter_z;
        }

        point_cloud_pub_->publish(*cloud_msg);
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Extract pitch and roll from IMU orientation quaternion
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(roll_, pitch_, yaw_);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;

    // IMU-derived orientation angles
    double pitch_;
    double roll_;
    double yaw_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MappingNode>());
    rclcpp::shutdown();
    return 0;
}