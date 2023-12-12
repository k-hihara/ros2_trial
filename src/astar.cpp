#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std;

struct Astar_Node {
    int x;
    int y;
    double cost;
    double heuristic;
    double totalCost;
    Astar_Node* parent;

    Astar_Node(int x, int y, double cost, double heuristic, Astar_Node* parent)
        : x(x), y(y), cost(cost), heuristic(heuristic), parent(parent) {
        totalCost = cost + heuristic;
    }

    Astar_Node() : x(0), y(0), cost(0), heuristic(0), totalCost(0), parent(nullptr) {}
};

class AStarPlanner : public rclcpp::Node {
public:
    AStarPlanner() : Node("astar_planner") {
        Astar_Node();
        start_subscriber_ = create_subscription<geometry_msgs::msg::Point>(
            "start", 10, std::bind(&AStarPlanner::startCallback, this, std::placeholders::_1));

        goal_subscriber_ = create_subscription<geometry_msgs::msg::Point>(
            "goal_pose", 10, std::bind(&AStarPlanner::goalCallback, this, std::placeholders::_1));

        costmap_subscriber_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "costmap", 10, std::bind(&AStarPlanner::costmapCallback, this, std::placeholders::_1));

        path_publisher_ = create_publisher<nav_msgs::msg::Path>("planned_path", 10);
    }

private:
    vector<vector<int>> costmap_;
    geometry_msgs::msg::Point start_;
    geometry_msgs::msg::Point goal_;

    struct CompareNodes {
        bool operator()(const Astar_Node* a, const Astar_Node* b) {
            return a->totalCost < b->totalCost;
        }
    };

    void startCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
        start_ = *msg;
    }

    void goalCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
        start_.x=50;start_.y=50;start_.z=0;
        goal_ = *msg;
        std::cout<<"goal callback"<<std::endl;
        planPath();
    }

    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        costmap_.resize(msg->info.width, vector<int>(msg->info.height, 0));
        for (int x = 0; x < msg->info.width; ++x) {
            for (int y = 0; y < msg->info.height; ++y) {
                costmap_[x][y] = msg->data[x + y * msg->info.width];
            }
        }
    }

    void planPath() {
        vector<Astar_Node*> openSet;
        vector<Astar_Node*> closedSet;

        Astar_Node* startNode = new Astar_Node(
            static_cast<int>(start_.x), static_cast<int>(start_.y), 0, calculateHeuristic(Astar_Node(), goal_), nullptr);

        openSet.push_back(startNode);

        while (!openSet.empty()) {
            auto current = min_element(openSet.begin(), openSet.end(), CompareNodes());
            Astar_Node* currentPtr = *current;

            openSet.erase(current);
            closedSet.push_back(currentPtr);

            if (currentPtr->x == static_cast<int>(goal_.x) && currentPtr->y == static_cast<int>(goal_.y)) {
                nav_msgs::msg::Path path;
                path.header.frame_id = "map";

                while (currentPtr != nullptr) {
                    geometry_msgs::msg::PoseStamped pose;
                    pose.pose.position.x = currentPtr->x;
                    pose.pose.position.y = currentPtr->y;
                    pose.pose.position.z = 0.0;
                    path.poses.push_back(pose);

                    currentPtr = currentPtr->parent;
                }

                path_publisher_->publish(path);

                for (auto node : openSet) {
                    delete node;
                }
                for (auto node : closedSet) {
                    delete node;
                }

                return;
            }

            exploreNeighbors(currentPtr, openSet, closedSet);
        }

        RCLCPP_WARN(get_logger(), "No valid path found");
    }

    void exploreNeighbors(Astar_Node* current, vector<Astar_Node*>& openSet, vector<Astar_Node*>& closedSet) {
        const vector<pair<int, int>> movements = {
            {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

        for (const auto& move : movements) {
            int newX = current->x + move.first;
            int newY = current->y + move.second;

            if (newX >= 0 && newX < costmap_.size() && newY >= 0 && newY < costmap_[0].size()) {
                if (costmap_[newX][newY] == 0) {//want to use threshould 
                    Astar_Node* neighbor = new Astar_Node(
                        newX, newY, current->cost + 1, calculateHeuristic(*current, goal_), current);

                    auto itClosed = find_if(closedSet.begin(), closedSet.end(), [neighbor](const Astar_Node* node) {
                        return node->x == neighbor->x && node->y == neighbor->y;
                    });

                    if (itClosed != closedSet.end()) {
                        delete neighbor;
                        continue;
                    }

                    auto itOpen = find_if(openSet.begin(), openSet.end(), [neighbor](const Astar_Node* node) {
                        return node->x == neighbor->x && node->y == neighbor->y;
                    });

                    if (itOpen == openSet.end()) {
                        openSet.push_back(neighbor);
                    } else {
                        if (neighbor->cost < (*itOpen)->cost) {
                            (*itOpen)->cost = neighbor->cost;
                            (*itOpen)->totalCost = neighbor->totalCost;
                            (*itOpen)->parent = neighbor->parent;
                            delete neighbor;
                        } else {
                            delete neighbor;
                        }
                    }
                }
            }
        }
    }

    double calculateHeuristic(const Astar_Node& from, const geometry_msgs::msg::Point& to) {
        return sqrt(pow(to.x - from.x, 2) + pow(to.y - from.y, 2));
    }

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr start_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goal_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AStarPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
