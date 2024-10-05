#include "rrt_planner/rrt_star_planner_node.hpp"

namespace rrt_star_planner {
RRTStarPlannerNode::RRTStarPlannerNode(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("rrt_star_planner_node", "", options) {
  this->declare_parameter("step_size", 1.0);
  this->declare_parameter("goal_tolerance", 0.5);
  this->declare_parameter("check_radius", 1.0);
  this->declare_parameter("rewire_radius", 2.0);

  rng_.seed(std::random_device{}());
  uniform_dist_ = std::uniform_real_distribution<>(0.0, 1.0);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RRTStarPlannerNode::on_configure(const rclcpp_lifecycle::State &) {
  step_size_ = this->get_parameter("step_size").as_double();
  goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
  check_radius_ = this->get_parameter("check_radius").as_double();
  rewire_radius_ = this->get_parameter("rewire_radius").as_double();

  map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", 10,
      std::bind(&RRTStarPlannerNode::map_callback, this,
                std::placeholders::_1));

  position_subscriber_ =
      this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "start_goal_position", 10,
          std::bind(&RRTStarPlannerNode::position_callback, this,
                    std::placeholders::_1));

  path_publisher_ =
      this->create_publisher<nav_msgs::msg::Path>("rrt_star_path", 10);
  start_marker_publisher_ =
      this->create_publisher<visualization_msgs::msg::Marker>("start_marker",
                                                              10);
  goal_marker_publisher_ =
      this->create_publisher<visualization_msgs::msg::Marker>("goal_marker",
                                                              10);

  RCLCPP_INFO(this->get_logger(), "RRTStarPlannerNode Configured.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RRTStarPlannerNode::on_activate(const rclcpp_lifecycle::State &) {
  lifecycle_bond_ = std::make_unique<LifecycleBond>(shared_from_this());
  path_publisher_->on_activate();
  start_marker_publisher_->on_activate();
  goal_marker_publisher_->on_activate();
  RCLCPP_INFO(this->get_logger(), "RRTStarPlannerNode Activated.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RRTStarPlannerNode::on_deactivate(const rclcpp_lifecycle::State &) {
  lifecycle_bond_.reset();
  path_publisher_->on_deactivate();
  start_marker_publisher_->on_deactivate();
  goal_marker_publisher_->on_deactivate();
  RCLCPP_INFO(this->get_logger(), "RRTStarPlannerNode deactivated.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RRTStarPlannerNode::on_cleanup(const rclcpp_lifecycle::State &) {
  path_publisher_.reset();
  start_marker_publisher_.reset();
  goal_marker_publisher_.reset();
  map_subscriber_.reset();
  position_subscriber_.reset();
  RCLCPP_INFO(this->get_logger(), "RRTStarPlannerNode cleaned up.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RRTStarPlannerNode::on_shutdown(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "RRTStarPlannerNode shutting down.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

void RRTStarPlannerNode::map_callback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {

  if (this->get_current_state().id() ==
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    map_ = msg;
    RCLCPP_INFO(this->get_logger(), "Map callback ..");
  }
}

void RRTStarPlannerNode::position_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {

  if (this->get_current_state().id() ==
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    if (msg->header.frame_id == "start") {
      start_position_.x() = msg->pose.position.x;
      start_position_.y() = msg->pose.position.y;
      RCLCPP_INFO(this->get_logger(), "Start position updated to (%f, %f)",
                  start_position_.x(), start_position_.y());
    } else if (msg->header.frame_id == "goal") {
      goal_position_.x() = msg->pose.position.x;
      goal_position_.y() = msg->pose.position.y;
      RCLCPP_INFO(this->get_logger(), "Goal position updated to (%f, %f)",
                  goal_position_.x(), goal_position_.y());
    }

    if (start_position_.x() != goal_position_.x() or
        start_position_.y() != goal_position_.y()) {
      compute_path();
    } else {
      RCLCPP_WARN(this->get_logger(), "Start and Goal positions are same");
    }
  }
}

void RRTStarPlannerNode::compute_path() {
  if (!map_) {
    RCLCPP_WARN(this->get_logger(), "Map is not available.");
    return;
  }

  // Use the received start and goal positions
  Vector2d start(start_position_.x(), start_position_.y());
  Vector2d goal(goal_position_.x(), goal_position_.y());

  publish_marker(start, "start_marker", visualization_msgs::msg::Marker::SPHERE,
                 0.0, 1.0, 0.0);
  publish_marker(goal, "goal_marker", visualization_msgs::msg::Marker::SPHERE,
                 1.0, 0.0, 0.0);

  std::vector<std::shared_ptr<TreeNode>> tree;
  auto start_node =
      std::make_shared<TreeNode>(TreeNode{start, nullptr, 0.0, 0});
  tree.push_back(start_node);

  bool path_found = false;
  std::shared_ptr<TreeNode> final_node = nullptr;

  int node_id = 0;

  for (int i = 0; i < 2000; ++i) {
    Vector2d random_point = get_random_point();
    auto nearest_node = get_nearest_node(tree, random_point);
    Vector2d new_position = extend(nearest_node->position, random_point);

    if (is_collision_free(nearest_node->position, new_position)) {
      auto new_node = std::make_shared<TreeNode>();
      new_node->position = new_position;
      new_node->parent = nearest_node;
      new_node->cost =
          nearest_node->cost + (new_position - nearest_node->position).norm();
      new_node->id = node_id++;

      tree.push_back(new_node);
      rewire(tree, new_node);

      if ((new_position - goal).norm() < goal_tolerance_) {
        path_found = true;
        final_node = new_node;
        break;
      }
    }
  }

  if (path_found) {
    RCLCPP_INFO(this->get_logger(), "Path found");
    std::vector<geometry_msgs::msg::PoseStamped> path;
    geometry_msgs::msg::PoseStamped pose;
    auto node = final_node;
    while (node->parent) {
      pose.pose.position.x = node->position.x();
      pose.pose.position.y = node->position.y();
      path.push_back(pose);
      node = node->parent;
    }
    pose.pose.position.x = node->position.x();
    pose.pose.position.y = node->position.y();
    path.push_back(pose);
    std::reverse(path.begin(), path.end());

    std::vector<Vector2d> vector_path;

    for (const auto &pose : path) {
      Vector2d point(pose.pose.position.x, pose.pose.position.y);
      vector_path.push_back(point);
    }

    publish_path(vector_path);

  } else {
    RCLCPP_WARN(this->get_logger(), "Failed to find a path.");
  }
}

void RRTStarPlannerNode::rewire(std::vector<std::shared_ptr<TreeNode>> &tree,
                                std::shared_ptr<TreeNode> new_node) {
  for (auto &node : tree) {
    if (node->id != new_node->id) {
      double distance = (node->position - new_node->position).norm();
      if (distance < rewire_radius_) {
        if (is_collision_free(node->position, new_node->position)) {
          double new_cost = new_node->cost + distance;
          if (new_cost < node->cost) {
            node->parent = new_node;
            node->cost = new_cost;
          }
        }
      }
    }
  }
}

Vector2d RRTStarPlannerNode::get_random_point() {
  double x = uniform_dist_(rng_) * map_->info.width * map_->info.resolution;
  double y = uniform_dist_(rng_) * map_->info.height * map_->info.resolution;
  return Vector2d(x, y);
}

std::shared_ptr<RRTStarPlannerNode::TreeNode>
RRTStarPlannerNode::get_nearest_node(
    const std::vector<std::shared_ptr<TreeNode>> &tree, const Vector2d &point) {
  double min_dist = std::numeric_limits<double>::max();
  std::shared_ptr<TreeNode> nearest_node = nullptr;

  for (const auto &node : tree) {
    double dist = (node->position - point).norm();
    if (dist < min_dist) {
      min_dist = dist;
      nearest_node = node;
    }
  }
  return nearest_node;
}

Vector2d RRTStarPlannerNode::extend(const Vector2d &from, const Vector2d &to) {
  Vector2d direction = to - from;
  direction.normalize();
  return from + step_size_ * direction;
}

bool RRTStarPlannerNode::is_collision_free(const Vector2d &from,
                                           const Vector2d &to) {
  auto map_to_grid = [&](const Vector2d &point) {
    int x = static_cast<int>(point.x() / map_->info.resolution);
    int y = static_cast<int>(point.y() / map_->info.resolution);
    return std::make_pair(x, y);
  };

  auto [x0, y0] = map_to_grid(from);
  auto [x1, y1] = map_to_grid(to);

  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  while (true) {
    if (!is_within_bounds(x0, y0) ||
        map_->data[y0 * map_->info.width + x0] > 50) {
      return false;
    }

    if (x0 == x1 && y0 == y1)
      break;
    int e2 = err * 2;
    if (e2 > -dy) {
      err -= dy;
      x0 += sx;
    }
    if (e2 < dx) {
      err += dx;
      y0 += sy;
    }
  }

  int radius =
      static_cast<int>(std::ceil(check_radius_ / map_->info.resolution));
  for (int dx = -radius; dx <= radius; ++dx) {
    for (int dy = -radius; dy <= radius; ++dy) {
      int nx = x0 + dx;
      int ny = y0 + dy;
      if (is_within_bounds(nx, ny) &&
          map_->data[ny * map_->info.width + nx] > 50) {
        return false;
      }
    }
  }

  return true;
}

bool RRTStarPlannerNode::is_within_bounds(int x, int y) {
  return x >= 0 && x < map_->info.width && y >= 0 && y < map_->info.height;
}

void RRTStarPlannerNode::publish_marker(const Vector2d &position,
                                        const std::string &frame_id,
                                        uint8_t shape, float r, float g,
                                        float b) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = this->now();
  marker.ns = "rrt_star";
  marker.id = 0;
  marker.type = shape;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = position.x();
  marker.pose.position.y = position.y();
  marker.pose.position.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;

  if (frame_id == "start_marker") {
    start_marker_publisher_->publish(marker);
  } else if (frame_id == "goal_marker") {
    goal_marker_publisher_->publish(marker);
  }
}

void RRTStarPlannerNode::publish_path(const std::vector<Vector2d> &path) {
  nav_msgs::msg::Path ros_path;
  ros_path.header.frame_id = "map";
  ros_path.header.stamp = this->now();
  for (const auto &point : path) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = this->now();
    pose.pose.position.x = point.x();
    pose.pose.position.y = point.y();
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    ros_path.poses.push_back(pose);
  }

  // Print the path for debugging
  std::cout << "Publishing Path: " << std::endl;
  for (const auto &pose : ros_path.poses) {
    std::cout << "Pose: (" << pose.pose.position.x << ", "
              << pose.pose.position.y << ", " << pose.pose.position.z << ")"
              << std::endl;
  }
  std::cout << "  ros_path.poses.size()" << ros_path.poses.size() << std::endl;
  path_publisher_->publish(ros_path);
}

void RRTStarPlannerNode::on_shutdown()
{
    auto current_state = get_current_state();

    if (current_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        on_deactivate(current_state);
    }
    if (current_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
        on_cleanup(current_state);
    }
    on_shutdown(current_state);
}

} // namespace rrt_star_planner

void on_shutdown(std::shared_ptr<rrt_star_planner::RRTStarPlannerNode> & rrt_star_planner_node)
{
  rrt_star_planner_node->on_shutdown();
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rrt_star_planner::RRTStarPlannerNode>();
  rclcpp::on_shutdown(std::bind(&on_shutdown, std::ref(node)));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
