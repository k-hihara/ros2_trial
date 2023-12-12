#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/segmentation/extract_clusters.h"

class PointCloudClusterNode : public rclcpp::Node
{
public:
  PointCloudClusterNode() : Node("pointcloud_cluster_node")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "random_pointcloud", 10, std::bind(&PointCloudClusterNode::pointCloudCallback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("clustered_pointcloud", 10);
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

    // Perform DBSCAN clustering here using PCL library
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(5); // set your cluster tolerance
    ec.setMinClusterSize(5);    // set your minimum cluster size
    ec.setMaxClusterSize(1000);   // set your maximum cluster size
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    pcl::PointCloud<pcl::PointXYZI>::Ptr intensity_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    int cluster_id = 0;
    for (const auto &indices : cluster_indices)
    {
      for (const auto &index : indices.indices)
      {
        pcl::PointXYZI point;
        point.x = cloud->points[index].x;
        point.y = cloud->points[index].y;
        point.z = cloud->points[index].z;

        // Set intensity based on cluster id
        point.intensity = static_cast<float>(cluster_id);

        intensity_cloud->points.push_back(point);
      }

      cluster_id++;
    }

    // Convert intensity_cloud to PointCloud2
    pcl::PCLPointCloud2 pcl_intensity_cloud;
    pcl::toPCLPointCloud2(*intensity_cloud, pcl_intensity_cloud);

    // Publish the clustered PointCloud2
    auto clustered_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl_conversions::moveFromPCL(pcl_intensity_cloud, *clustered_msg);
    clustered_msg->header.frame_id = "base_link";
    publisher_->publish(std::move(clustered_msg));

    // Print the number of clusters to the console
    RCLCPP_INFO(this->get_logger(), "Number of clusters: %d", cluster_id);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudClusterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
