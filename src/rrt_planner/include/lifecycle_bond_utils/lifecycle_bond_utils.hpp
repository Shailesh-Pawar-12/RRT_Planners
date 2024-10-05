#include "bondcpp/bond.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace lifecycle_bond
{

class LifecycleBond
{
private:
  std::unique_ptr<bond::Bond> bond_{nullptr};
  rclcpp_lifecycle::LifecycleNode::SharedPtr lifecycle_node_;

public:
  LifecycleBond(rclcpp_lifecycle::LifecycleNode::SharedPtr lifecycle_node);
  ~LifecycleBond();
  void create_bond();
  void destroy_bond();
};

}  // namespace lifecycle_bond
