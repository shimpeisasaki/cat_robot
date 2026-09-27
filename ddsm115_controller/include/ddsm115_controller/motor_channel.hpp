#pragma once
#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include "ddsm115_controller/motor_control.hpp"
namespace ddsm115_controller {
struct MotorChannel
{
  MotorChannel(
    std::string label, int id, std::unique_ptr<MotorControl> motor,
    bool freewheel_on_shutdown)
  : name(std::move(label)), motor_id(id), driver(std::move(motor)),
    freewheel_on_shutdown(freewheel_on_shutdown) {}
  MotorChannel(MotorChannel &&) = default;
  MotorChannel(const MotorChannel &) = delete;
  ~MotorChannel()
  {
    // Also runs on partial construction failure, exception, and normal SIGINT exit.
    // Does not depend on ROS publishers or a running ROS context.
    if (!driver) {return;}
    if (freewheel_on_shutdown) {
      for (int attempt = 0; attempt < 3; ++attempt) {
        try {
          driver->send_current(motor_id, 0.0F);
          driver->set_drive_mode(motor_id, 1);
          driver->send_current(motor_id, 0.0F);
          return;
        } catch (const std::exception & error) {
          std::fprintf(
            stderr, "Shutdown freewheel %s ID %d: %s\n", name.c_str(), motor_id,
            error.what());
        }
      }
      std::fprintf(
        stderr, "Shutdown freewheel %s ID %d: not confirmed; falling back to brake\n",
        name.c_str(), motor_id);
    }
    for (int attempt = 0; attempt < 3; ++attempt) {
      try {
        driver->send_current(motor_id, 0.0F);  // zero in either current or velocity mode
        driver->set_drive_mode(motor_id, 2);
        const auto reply = driver->set_brake(motor_id);
        if (reply.id == motor_id && reply.rpm == 0 && reply.error == 0) {
          return;
        }
      } catch (const std::exception & error) {
        std::fprintf(stderr, "Shutdown brake %s ID %d: %s\n", name.c_str(), motor_id, error.what());
      }
    }
    std::fprintf(stderr, "Shutdown brake %s ID %d: stop NOT confirmed; check physical stop\n",
      name.c_str(), motor_id);
  }
  std::string name;
  int motor_id;
  std::unique_ptr<MotorControl> driver;
  bool freewheel_on_shutdown;
};

}
