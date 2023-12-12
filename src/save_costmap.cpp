#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <fstream>

class CostmapSubscriber : public rclcpp::Node {
public:
    CostmapSubscriber() : Node("costmap_subscriber") {
        costmap_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "costmap", 10, std::bind(&CostmapSubscriber::costmapCallback, this, std::placeholders::_1));
    }

private:
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        // Save costmap data to YAML file
        saveCostmapToYAML(msg, "costmap.yaml");

        // Shutdown the node after receiving the costmap once
        rclcpp::shutdown();
    }

    void saveCostmapToYAML(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap, const std::string& filename) {
        std::ofstream file(filename);

        file << "header:" << std::endl;
        file << "  frame_id: " << costmap->header.frame_id << std::endl;
        file << "info:" << std::endl;
        file << "  resolution: " << costmap->info.resolution << std::endl;
        file << "  width: " << costmap->info.width << std::endl;
        file << "  height: " << costmap->info.height << std::endl;
        file << "  origin:" << std::endl;
        file << "    position:" << std::endl;
        file << "      x: " << costmap->info.origin.position.x << std::endl;
        file << "      y: " << costmap->info.origin.position.y << std::endl;
        file << "      z: " << costmap->info.origin.position.z << std::endl;
        file << "    orientation:" << std::endl;
        file << "      x: " << costmap->info.origin.orientation.x << std::endl;
        file << "      y: " << costmap->info.origin.orientation.y << std::endl;
        file << "      z: " << costmap->info.origin.orientation.z << std::endl;
        file << "      w: " << costmap->info.origin.orientation.w << std::endl;
        file << "data:" << std::endl;

        for (int i = 0; i < costmap->info.width * costmap->info.height; ++i) {
            file << "  - " << static_cast<int>(costmap->data[i]) << std::endl;
        }

        RCLCPP_INFO(get_logger(), "Costmap data saved to %s", filename.c_str());
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_subscription_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CostmapSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

