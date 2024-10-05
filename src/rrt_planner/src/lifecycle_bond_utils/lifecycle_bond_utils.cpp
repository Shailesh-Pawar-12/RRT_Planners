#include <lifecycle_bond_utils/lifecycle_bond_utils.hpp>

namespace lifecycle_bond
{

LifecycleBond::LifecycleBond(rclcpp_lifecycle::LifecycleNode::SharedPtr lifecycle_node)
: lifecycle_node_(lifecycle_node)
{
  create_bond();
}

LifecycleBond::~LifecycleBond() { destroy_bond(); }

void LifecycleBond::create_bond()
{
  RCLCPP_INFO(
    lifecycle_node_->get_logger(), "Creating bond (%s) to lifecycle manager.",
    lifecycle_node_->get_name());

  bond_ = std::make_unique<bond::Bond>(
    std::string("bond"), lifecycle_node_->get_name(),
    lifecycle_node_->shared_from_this()); 
                                          
  bond_->setHeartbeatPeriod(0.1);  
  bond_->setHeartbeatTimeout(
    10.0);  
  bond_->start();
}

void LifecycleBond::destroy_bond()
{
  RCLCPP_INFO(
    lifecycle_node_->get_logger(), "Destroying bond (%s) to lifecycle manager.",
    lifecycle_node_->get_name());
  if (bond_)
  {
    bond_.reset();
  }
}
}  // namespace lifecycle_bond
