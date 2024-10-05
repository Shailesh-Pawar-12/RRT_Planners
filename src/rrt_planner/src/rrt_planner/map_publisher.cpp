#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

class StaticMapPublisherNode : public rclcpp::Node {
public:
  StaticMapPublisherNode() : Node("static_map_publisher") {
    map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(rclcpp::KeepLast(10)).transient_local());

    // Set up the parameters of the map
    map_.header.frame_id = "map";
    map_.info.resolution = 1;
    map_.info.width = 20;  // 20 cells wide
    map_.info.height = 20; // 20 cells high
    map_.info.origin.position.x =
        0.0; // Bottom-left corner of the map in world coordinates
    map_.info.origin.position.y = 0.0;
    map_.info.origin.position.z = 0.0;

    // Initialize map data with free space (-1: unknown, 0: free, 100: occupied)
    map_.data.resize(map_.info.width * map_.info.height, 0);

    // Add a static obstacle (e.g., a wall)
    bool toggle = false;
    for (int j = 5; j < map_.info.height - 2; j += 5) {
      if (toggle) {
        for (int i = map_.info.width - 1; i > 2; i -= 1) {
          addWall(map_, i, j);
          toggle = false;
        }
      } else {
        for (int i = 0; i < map_.info.width - 2; i += 1) {
          addWall(map_, i, j);
          toggle = true;
        }
      }
    }

    
    // Publish map periodically
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&StaticMapPublisherNode::publishMap, this));
  }

  void addWall(nav_msgs::msg::OccupancyGrid &map_, int start_x, int start_y) {
    for (int y = start_y; y < start_y + wall_height; ++y) {
      for (int x = start_x; x < start_x + wall_width; ++x) {
        int index = y * map_.info.width + x;
        map_.data[index] = 100; // Set as occupied space
      }
    }
  }

private:
  void publishMap() {
    map_.header.stamp = this->now(); // Update timestamp
    map_publisher_->publish(map_);   // Publish the map
    std::cout << " Publishing map info ..." << std::endl;
  }

  nav_msgs::msg::OccupancyGrid map_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int wall_width = 1;  // 1 cells wide
  int wall_height = 1; // 1 cells high
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticMapPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
