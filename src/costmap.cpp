#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class CostmapPublisher : public rclcpp::Node {
public:
    CostmapPublisher() : Node("costmap_publisher") {
        costmap_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>("costmap", 10);
        timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&CostmapPublisher::publishCostmap, this));
    }

private:
    void publishCostmap() {
        nav_msgs::msg::OccupancyGrid costmap;
        costmap.header.stamp = now();
        costmap.header.frame_id = "map";
        costmap.info.width = 100;
        costmap.info.height = 100;
        costmap.info.resolution = 1.0;

        // Initialize all cells to 0
        costmap.data.assign(costmap.info.width * costmap.info.height, 0);

        costmap_publisher_->publish(costmap);
    }

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CostmapPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
