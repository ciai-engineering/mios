// Direct libfranka transport diagnostic.
//
// This intentionally contains no motion generator, target pose, velocity, or
// gripper operation. libfranka/FCI supplies gravity compensation while the
// callback writes only zero external joint torques for the requested duration.

#include <array>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <string>

#include <franka/control_types.h>
#include <franka/exception.h>
#include <franka/robot.h>

namespace {

constexpr std::array<double, 7> kZeroTorque{};

double parse_duration_seconds(const char* value) {
  try {
    const double seconds = std::stod(value);
    if (seconds > 0.0 && seconds <= 120.0) {
      return seconds;
    }
  } catch (const std::exception&) {
  }
  throw std::invalid_argument("duration must be greater than zero and at most 120 seconds");
}

}  // namespace

int main(int argc, char** argv) {
  const std::string robot_ip = argc >= 2 ? argv[1] : "192.168.4.100";
  const double duration_seconds = argc >= 3 ? parse_duration_seconds(argv[2]) : 90.0;

  try {
    std::cout << "Connecting to " << robot_ip << "..." << std::endl;
    franka::Robot robot(robot_ip, franka::RealtimeConfig::kEnforce);
    std::cout << "Starting direct zero-torque transport test for " << duration_seconds
              << " seconds; no motion target or gripper command will be sent." << std::endl;

    double elapsed_seconds = 0.0;
    std::size_t cycles = 0;
    robot.control(
        [&](const franka::RobotState&, franka::Duration period) {
          elapsed_seconds += period.toSec();
          ++cycles;
          franka::Torques zero_torque(kZeroTorque);
          if (elapsed_seconds >= duration_seconds) {
            return franka::MotionFinished(zero_torque);
          }
          return zero_torque;
        },
        false, franka::kMaxCutoffFrequency);

    std::cout << "PASS: completed " << cycles << " zero-torque FCI cycles in "
              << elapsed_seconds << " seconds." << std::endl;
    return EXIT_SUCCESS;
  } catch (const franka::Exception& error) {
    std::cerr << "FAIL: libfranka error: " << error.what() << std::endl;
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << std::endl;
  }
  return EXIT_FAILURE;
}
