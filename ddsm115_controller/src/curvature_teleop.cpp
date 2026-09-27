#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace ddsm115_controller
{

class CurvatureTeleop : public rclcpp::Node
{
public:
  CurvatureTeleop()
  : Node("curvature_teleop"),
    update_period_(declare_parameter("update_period", 0.02)),
    joystick_timeout_(declare_parameter("joystick_timeout", 0.5)),
    joystick_deadzone_(declare_parameter("joystick_deadzone", 0.03)),
    curvature_deadzone_(declare_parameter("curvature_deadzone", 0.05)),
    normal_max_linear_speed_(declare_parameter("normal_max_linear_speed", 1.0)),
    maximum_curvature_(declare_parameter("maximum_curvature", 0.8))
  {
    if (update_period_ <= 0.0 || joystick_timeout_ <= 0.0 ||
      joystick_deadzone_ < 0.0 || joystick_deadzone_ >= 1.0 ||
      curvature_deadzone_ < 0.0 || curvature_deadzone_ >= 1.0 ||
      normal_max_linear_speed_ <= 0.0 ||
      maximum_curvature_ <= 0.0)
    {
      throw std::invalid_argument("Invalid curvature teleop parameter");
    }

    command_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_teleop", 10);
    joystick_subscription_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&CurvatureTeleop::joystick_command, this, std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(update_period_)),
      std::bind(&CurvatureTeleop::publish_command, this));

    RCLCPP_INFO(
      get_logger(), "Curvature teleop: max %.3f m/s, max curvature %.3f 1/m",
      normal_max_linear_speed_, maximum_curvature_);
  }

private:
  static double apply_deadzone(double value, double deadzone)
  {
    value = std::clamp(value, -1.0, 1.0);
    if (std::abs(value) <= deadzone) {
      return 0.0;
    }
    return std::copysign((std::abs(value) - deadzone) / (1.0 - deadzone), value);
  }

  void joystick_command(const sensor_msgs::msg::Joy & message)
  {
    last_joystick_time_ = std::chrono::steady_clock::now();
    const double right_horizontal = message.axes.size() > 3 ? message.axes[3] : 0.0;
    const double right_vertical = message.axes.size() > 4 ? message.axes[4] : 0.0;
    linear_input_ = apply_deadzone(right_vertical, joystick_deadzone_);
    curvature_input_ = apply_deadzone(right_horizontal, curvature_deadzone_);

    const auto pressed = [&message, this](std::size_t index) {
        return message.buttons.size() > index && message.buttons[index] == 1 &&
               (previous_buttons_.size() <= index || previous_buttons_[index] == 0);
      };
    if (pressed(1)) {  // B: Brake has priority over simultaneous Y
      drive_enabled_ = false;
    } else if (pressed(3)) {  // Y: Freewheel
      drive_enabled_ = false;
    } else if (pressed(0)) {  // A: external command source
      drive_enabled_ = false;
    } else if (pressed(2)) {  // X: manual command source
      drive_enabled_ = true;
    }
    previous_buttons_ = message.buttons;
  }

  void publish_command()
  {
    const auto now = std::chrono::steady_clock::now();
    const double seconds_since_joystick =
      std::chrono::duration<double>(now - last_joystick_time_).count();
    const bool joystick_online = seconds_since_joystick <= joystick_timeout_;
    geometry_msgs::msg::Twist command;
    if (drive_enabled_ && joystick_online) {
      command.linear.x = linear_input_ * normal_max_linear_speed_;
      // Curvature drive: no translational command means no normal-mode spin.
      command.angular.z = command.linear.x * curvature_input_ * maximum_curvature_;
    }
    command_publisher_->publish(command);
  }

  double update_period_;
  double joystick_timeout_;
  double joystick_deadzone_;
  double curvature_deadzone_;
  double normal_max_linear_speed_;
  double maximum_curvature_;
  bool drive_enabled_{false};
  double linear_input_{0.0};
  double curvature_input_{0.0};
  std::vector<int32_t> previous_buttons_;
  std::chrono::steady_clock::time_point last_joystick_time_{};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ddsm115_controller

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ddsm115_controller::CurvatureTeleop>());
  rclcpp::shutdown();
  return 0;
}
