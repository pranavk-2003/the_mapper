#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PointCloudSaver : public rclcpp::Node
{
public:
    PointCloudSaver() : Node("pointcloud_saver")
    {
        // Subscription to the point cloud topic
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/octomap_point_cloud_centers", 10,
            std::bind(&PointCloudSaver::pointCloudCallback, this, std::placeholders::_1));
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert ROS PointCloud2 message to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        // Save the PCL PointCloud as a PCD file
        std::string filename = "octomap_point_cloud_centers.pcd";
        if (pcl::io::savePCDFileASCII(filename, cloud) == 0)
        {
            RCLCPP_INFO(this->get_logger(), "Saved point cloud to %s", filename.c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to save point cloud to %s", filename.c_str());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudSaver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
