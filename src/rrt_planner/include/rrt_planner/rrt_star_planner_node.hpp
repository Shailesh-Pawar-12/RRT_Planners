#ifndef RRT_STAR_PLANNER_NODE_HPP
#define RRT_STAR_PLANNER_NODE_HPP

#include "rrt_planner/common.hpp"

using namespace Eigen;
using LifecycleBond = lifecycle_bond::LifecycleBond;
namespace rrt_star_planner{
class RRTStarPlannerNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit RRTStarPlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  void on_shutdown();
protected:
  // Lifecycle Node lifecycle callbacks
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &);

private:
  struct TreeNode {
    Vector2d position;
    std::shared_ptr<TreeNode> parent;
    double cost;
    int id;
  };

  Vector2d start_position_{0.0,0.0};
  Vector2d goal_position_{0.0,0.0};

  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void compute_path();
  void rewire(std::vector<std::shared_ptr<TreeNode>> &tree, std::shared_ptr<TreeNode> new_node);
  Vector2d get_random_point();
  std::shared_ptr<TreeNode> get_nearest_node(const std::vector<std::shared_ptr<TreeNode>> &tree, const Vector2d &point);
  Vector2d extend(const Vector2d &from, const Vector2d &to);
  bool is_collision_free(const Vector2d &from, const Vector2d &to);
  bool is_within_bounds(int x, int y);
  void publish_marker(const Vector2d &position, const std::string &frame_id, uint8_t shape, float r, float g, float b);
  void publish_path(const std::vector<Vector2d> &path);
  void position_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  double step_size_;
  double goal_tolerance_;
  double check_radius_;
  double rewire_radius_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_subscriber_;  
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr start_marker_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_publisher_;

  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  std::default_random_engine rng_;
  std::uniform_real_distribution<> uniform_dist_;

  std::unique_ptr<LifecycleBond> lifecycle_bond_{nullptr};
};
}
#endif // RRT_STAR_PLANNER_NODE_HPP
