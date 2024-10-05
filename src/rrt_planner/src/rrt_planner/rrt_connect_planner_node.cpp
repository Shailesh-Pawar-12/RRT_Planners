#include "rrt_planner/rrt_connect_planner_node.hpp"

namespace rrt_connect_planner {

RRTConnectPlannerNode::RRTConnectPlannerNode(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("rrt_connect_planner_node", "", options) {
  this->declare_parameter("step_size", 1.0);
  this->declare_parameter("goal_tolerance", 0.5);
  this->declare_parameter("check_radius", 0.5);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RRTConnectPlannerNode::on_configure(const rclcpp_lifecycle::State &) {

  step_size_ = this->get_parameter("step_size").as_double();
  goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
  check_radius_ = this->get_parameter("check_radius").as_double();

  map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", 10,
      std::bind(&RRTConnectPlannerNode::map_callback, this,
                std::placeholders::_1));

  position_subscriber_ =
      this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "start_goal_position", 10,
          std::bind(&RRTConnectPlannerNode::position_callback, this,
                    std::placeholders::_1));

  path_publisher_ =
      this->create_publisher<nav_msgs::msg::Path>("rrt_connect_path", 10);
  start_marker_publisher_ =
      this->create_publisher<visualization_msgs::msg::Marker>("start_marker",
                                                              10);
  goal_marker_publisher_ =
      this->create_publisher<visualization_msgs::msg::Marker>("goal_marker",
                                                              10);

  rng_.seed(std::random_device{}());
  uniform_dist_ = std::uniform_real_distribution<>(0.0, 1.0);

  auto result = this->add_on_set_parameters_callback(
      std::bind(&RRTConnectPlannerNode::parameters_callback, this,
                std::placeholders::_1));
  (void)result;

  RCLCPP_INFO(this->get_logger(), "RRTConnect Planner Configured");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RRTConnectPlannerNode::on_cleanup(const rclcpp_lifecycle::State &) {

  path_publisher_.reset();
  start_marker_publisher_.reset();
  goal_marker_publisher_.reset();
  map_subscriber_.reset();
  position_subscriber_.reset();
  RCLCPP_INFO(this->get_logger(), "RRTConnect Planner Cleaned up");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RRTConnectPlannerNode::on_deactivate(const rclcpp_lifecycle::State &) {
  path_publisher_->on_deactivate();
  start_marker_publisher_->on_deactivate();
  goal_marker_publisher_->on_deactivate();
  RCLCPP_INFO(this->get_logger(), "RRTConnect Planner Deactivated");
  lifecycle_bond_.reset();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RRTConnectPlannerNode::on_activate(const rclcpp_lifecycle::State &) {
  lifecycle_bond_ = std::make_unique<LifecycleBond>(shared_from_this());
  path_publisher_->on_activate();
  start_marker_publisher_->on_activate();
  goal_marker_publisher_->on_activate();
  RCLCPP_INFO(this->get_logger(), "RRTConnect Planner Activated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RRTConnectPlannerNode::on_shutdown(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "RRTConnect Planner Shutdown");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rcl_interfaces::msg::SetParametersResult
RRTConnectPlannerNode::parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters) {
  auto result = rcl_interfaces::msg::SetParametersResult();
  for (const auto &param : parameters) {
    if (param.get_name() == "step_size") {
      step_size_ = param.as_double();
    } else if (param.get_name() == "goal_tolerance") {
      goal_tolerance_ = param.as_double();
    } else if (param.get_name() == "check_radius") {
      check_radius_ = param.as_double();
    }
  }
  result.successful = true;
  return result;
}

void RRTConnectPlannerNode::map_callback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  if (this->get_current_state().id() ==
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    map_ = msg;
    RCLCPP_INFO(this->get_logger(), "Map callback called.");
  }
}

void RRTConnectPlannerNode::position_callback(
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
void RRTConnectPlannerNode::compute_path() {
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

  std::vector<std::shared_ptr<TreeNode>> tree_start;
  std::vector<std::shared_ptr<TreeNode>> tree_goal;

  auto start_node = std::make_shared<TreeNode>(TreeNode{start, nullptr, 0});
  tree_start.push_back(start_node);

  auto goal_node = std::make_shared<TreeNode>(TreeNode{goal, nullptr, 1});
  tree_goal.push_back(goal_node);

  bool path_found = false;
  std::shared_ptr<TreeNode> final_node_start = nullptr;
  std::shared_ptr<TreeNode> final_node_goal = nullptr;

  int node_id_start = 0;
  int node_id_goal = 1;

  for (int i = 0; i < 2000; ++i) {
    Vector2d random_point = get_random_point();

    // Expand tree from start
    auto nearest_node_start = get_nearest_node(tree_start, random_point);
    Vector2d new_position_start =
        extend(nearest_node_start->position, random_point);

    if (is_collision_free(nearest_node_start->position, new_position_start)) {
      auto new_node_start = std::make_shared<TreeNode>();
      new_node_start->position = new_position_start;
      new_node_start->parent = nearest_node_start;
      new_node_start->id = node_id_start++;

      RCLCPP_INFO(this->get_logger(),
                  "Start tree: Collision free from nearest node to new "
                  "position. Nearest Node Position: (%f, %f), New Position: "
                  "(%f, %f), Distance to Goal: %f",
                  nearest_node_start->position.x(),
                  nearest_node_start->position.y(), new_position_start.x(),
                  new_position_start.y(), (new_position_start - goal).norm());

      tree_start.push_back(new_node_start);

      // Check if the new position is close to the goal tree
      auto nearest_node_goal = get_nearest_node(tree_goal, new_position_start);
      if ((new_position_start - nearest_node_goal->position).norm() <
          goal_tolerance_) {
        path_found = true;
        RCLCPP_INFO(this->get_logger(), "Changed path_found from start.");
        final_node_start = new_node_start;
        final_node_goal = nearest_node_goal;
        break;
      }
    }

    // Expand tree from goal
    auto nearest_node_goal = get_nearest_node(tree_goal, random_point);
    Vector2d new_position_goal =
        extend(nearest_node_goal->position, random_point);

    if (is_collision_free(nearest_node_goal->position, new_position_goal)) {
      auto new_node_goal = std::make_shared<TreeNode>();
      new_node_goal->position = new_position_goal;
      new_node_goal->parent = nearest_node_goal;
      new_node_goal->id = node_id_goal++;

      RCLCPP_INFO(this->get_logger(),
                  "Goal tree: Collision free from nearest node to new "
                  "position. Nearest Node Position: (%f, %f), New Position: "
                  "(%f, %f), Distance to Start: %f",
                  nearest_node_goal->position.x(),
                  nearest_node_goal->position.y(), new_position_goal.x(),
                  new_position_goal.y(), (new_position_goal - start).norm());

      tree_goal.push_back(new_node_goal);

      // Check if the new position is close to the start tree
      auto nearest_node_start = get_nearest_node(tree_start, new_position_goal);
      if ((new_position_goal - nearest_node_start->position).norm() <
          goal_tolerance_) {
        path_found = true;
        RCLCPP_INFO(this->get_logger(), "Changed path_found from goal.");
        final_node_start = nearest_node_start;
        final_node_goal = new_node_goal;
        break;
      }
    }
  }

  if (path_found) {
    RCLCPP_INFO(this->get_logger(), "Path found!");
    std::vector<geometry_msgs::msg::PoseStamped> path;

    auto node = final_node_start;
    while (node != nullptr) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = node->position.x();
      pose.pose.position.y = node->position.y();
      path.push_back(pose);
      node = node->parent;
    }

    std::reverse(path.begin(), path.end());

    auto goal_to_start_node = final_node_goal;
    while (goal_to_start_node != nullptr) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = goal_to_start_node->position.x();
      pose.pose.position.y = goal_to_start_node->position.y();
      path.push_back(pose);
      goal_to_start_node = goal_to_start_node->parent;
    }

    std::vector<Vector2d> vector_path;

    for (const auto &pose : path) {
      Vector2d point(pose.pose.position.x, pose.pose.position.y);
      vector_path.push_back(point);
    }

    publish_path(vector_path);
    std::cout << "-----------------------------------------path publish "
                 "-----------------"
              << std::endl;

  } else {
    RCLCPP_WARN(this->get_logger(), "No path found.");
  }
}

void RRTConnectPlannerNode::print_path(const std::vector<Vector2d> &path) {
  RCLCPP_INFO(this->get_logger(), "Path found:");
  for (const auto &point : path) {
    RCLCPP_INFO(this->get_logger(), "(%f, %f)", point.x(), point.y());
  }
}

void RRTConnectPlannerNode::publish_path(const std::vector<Vector2d> &path) {
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

  path_publisher_->publish(ros_path);
}

Vector2d RRTConnectPlannerNode::get_random_point() {
  double x = uniform_dist_(rng_) * map_->info.width * map_->info.resolution;
  double y = uniform_dist_(rng_) * map_->info.height * map_->info.resolution;
  return Vector2d(x, y);
}

std::shared_ptr<RRTConnectPlannerNode::TreeNode>
RRTConnectPlannerNode::get_nearest_node(
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

Vector2d RRTConnectPlannerNode::extend(const Vector2d &start,
                                       const Vector2d &end) {
  Vector2d direction = end - start;
  double length = direction.norm();
  if (length > step_size_) {
    direction = (direction / length) * step_size_;
  }
  return start + direction;
}

bool RRTConnectPlannerNode::is_collision_free(const Vector2d &start,
                                              const Vector2d &end) {
  double margin = 0.5;

  int x0 = static_cast<int>(start.x() / map_->info.resolution);
  int y0 = static_cast<int>(start.y() / map_->info.resolution);
  int x1 = static_cast<int>(end.x() / map_->info.resolution);
  int y1 = static_cast<int>(end.y() / map_->info.resolution);

  // Handle out of bounds
  if (x0 < 0 || x0 >= map_->info.width || y0 < 0 || y0 >= map_->info.height ||
      x1 < 0 || x1 >= map_->info.width || y1 < 0 || y1 >= map_->info.height) {
    return false;
  }

  // Check each point along the line
  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  while (true) {
    int index = y0 * map_->info.width + x0;
    if (map_->data[index] > 50) { // Assuming value > 50 indicates obstacle
      return false;
    }

    // Check surrounding cells within the margin
    for (double i = -margin; i <= margin; i += map_->info.resolution) {
      for (double j = -margin; j <= margin; j += map_->info.resolution) {
        int check_x = static_cast<int>((start.x() + i) / map_->info.resolution);
        int check_y = static_cast<int>((start.y() + j) / map_->info.resolution);

        if (check_x >= 0 && check_x < map_->info.width && check_y >= 0 &&
            check_y < map_->info.height) {
          int check_index = check_y * map_->info.width + check_x;
          if (map_->data[check_index] > 50) {
            return false;
          }
        }
      }
    }

    if (x0 == x1 && y0 == y1) {
      break;
    }
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

  return true;
}

bool RRTConnectPlannerNode::connect_trees(
    const std::vector<std::shared_ptr<TreeNode>> &tree_start,
    const std::vector<std::shared_ptr<TreeNode>> &tree_goal, double tolerance) {
  return true;
}

void RRTConnectPlannerNode::print_tree(
    const std::vector<std::shared_ptr<TreeNode>> &tree,
    const std::string &tree_name) {
  RCLCPP_INFO(this->get_logger(), "Printing %s tree:", tree_name.c_str());
  for (const auto &node : tree) {
    RCLCPP_INFO(this->get_logger(), "Node ID: %d, Position: (%f, %f)", node->id,
                node->position.x(), node->position.y());
  }
  RCLCPP_INFO(this->get_logger(), "End of %s tree.", tree_name.c_str());
}

void RRTConnectPlannerNode::publish_marker(const Vector2d &position,
                                           const std::string &frame_id,
                                           uint8_t type, float r, float g,
                                           float b) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = this->now();
  marker.ns = frame_id;
  marker.id = 0;
  marker.type = type;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = position.x();
  marker.pose.position.y = position.y();
  marker.pose.position.z = 0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
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


void RRTConnectPlannerNode::on_shutdown()
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
} // namespace rrt_connect_planner

void on_shutdown(std::shared_ptr<rrt_connect_planner::RRTConnectPlannerNode> & rrt_connect_planner_node)
{
  rrt_connect_planner_node->on_shutdown();
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rrt_connect_planner::RRTConnectPlannerNode>();
  rclcpp::on_shutdown(std::bind(&on_shutdown, std::ref(node)));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
