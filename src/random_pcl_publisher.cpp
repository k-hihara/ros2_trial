#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

class PointCloudPublisherNode : public rclcpp::Node
{
public:
  PointCloudPublisherNode() : Node("pointcloud_publisher_node")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("random_pointcloud", 10);
    timer_ = this->create_wall_timer(std::chrono::seconds(10), std::bind(&PointCloudPublisherNode::publishRandomPointCloud, this));
  }

private:
  void publishRandomPointCloud()
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Generate 1000 random points in the specified range
    for (int i = 0; i < 1000; ++i)
    {
      pcl::PointXYZ point;
      point.x = static_cast<float>(rand()) / RAND_MAX * 200 - 100; // Range: -100 to 100
      point.y = static_cast<float>(rand()) / RAND_MAX * 200 - 100; // Range: -100 to 100
      point.z = static_cast<float>(rand()) / RAND_MAX * 11 - 1;     // Range: -1 to 10
      cloud->points.push_back(point);
    }

    // Convert PointCloud to PointCloud2
    sensor_msgs::msg::PointCloud2::UniquePtr msg(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*cloud, *msg);
    msg->header.frame_id = "base_link"; // Set your desired frame_id

    // Publish the PointCloud2 message
    publisher_->publish(std::move(msg));
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
