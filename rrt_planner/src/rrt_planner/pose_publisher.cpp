#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class PosePublisherNode : public rclcpp::Node
{
public:
  PosePublisherNode() : Node("pose_publisher_node")
  {
    this->declare_parameter<std::string>("start_frame_id", "start");
    this->declare_parameter<std::string>("goal_frame_id", "goal");

    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("start_goal_position", 10);

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&PosePublisherNode::publish_pose, this)
    );
  }

private:
  void publish_pose()
  {

    auto start_pose = geometry_msgs::msg::PoseStamped();
    start_pose.header.stamp = this->now();
    start_pose.header.frame_id = this->get_parameter("start_frame_id").as_string();
    start_pose.pose.position.x = 1.0;
    start_pose.pose.position.y = 1.0;
    start_pose.pose.position.z = 0.0;
    start_pose.pose.orientation.w = 1.0;

    auto goal_pose = geometry_msgs::msg::PoseStamped();
    goal_pose.header.stamp = this->now();
    goal_pose.header.frame_id = this->get_parameter("goal_frame_id").as_string();
    goal_pose.pose.position.x = 10.0;
    goal_pose.pose.position.y = 9.0;
    goal_pose.pose.position.z = 0.0;
    goal_pose.pose.orientation.w = 1.0;

    // Publish the messages
    pose_publisher_->publish(start_pose);
    pose_publisher_->publish(goal_pose);

    RCLCPP_INFO(this->get_logger(), "Published start position: (%f, %f)",
                start_pose.pose.position.x, start_pose.pose.position.y);
    RCLCPP_INFO(this->get_logger(), "Published goal position: (%f, %f)",
                goal_pose.pose.position.x, goal_pose.pose.position.y);
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PosePublisherNode>());
  rclcpp::shutdown();
  return 0;
}
