# ros2_trial
ros2 実装練習用

## 環境
Ubuntu22.04 : ROS2 Humble

## 中身
### astar
-input geometry_msgs::msg::Point "start", geometry_msgs::msg::Point "goal_pose", nav_msgs::msg::OccupancyGrid "costmap"
-output nav_msgs::msg::Path "planned_path" 

### costmap
astar検証用。1秒おきに中身が全部0のコストマップを発行, 適当なyamlを作って読み込むようにしよう
-output nav_msgs::msg::OccupancyGrid "costmap" 

### save_cost_map
"coostmap"トピックを一度のみ読み込んでyamlを書き出す,なにかの点群データからcostmapを作ろう

##Usage
コストマップとプランナnode起動
```bash
ros2 run trial_ros2_pkg costmap
ros2 run trial_ros2_pkg astar_planner
```

適当なゴールをコマンドでパブリッシュ
```bash
ros2 topic pub /goal_pose geometry_msgs/msg/Point "{x: 4.0, y: 1.0, z: 0.0}"
```