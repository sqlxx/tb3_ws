#include "nav2_bt_navigator/bt_navigator.hpp"
#include "rclcpp/rclcpp.hpp"

namespace tb3_navigation
{
  class TourNavigator : public nav2_bt_navigator::Navigator<nav2_msgs::action::NavigateToPose>
  {
  public:
    using ActionT = nav2_msgs::action::NavigateToPose;
    TourNavigator() : Navigator() {};

    bool configure(rclcpp_lifecycle::LifecycleNode::WeakPtr node, std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) override {

      RCLCPP_INFO(rclcpp::get_logger("TourNavigator"), "TourNavigator configured");
      return true;
    }


  };
}

#include "pluginlib/class_list_macros.hpp"
// PLUGINLIB_EXPORT_CLASS(tb3_navigation::TourNavigator, nav2_bt_navigator::Navigator)


