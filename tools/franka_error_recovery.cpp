#include <franka/exception.h>
#include <franka/robot.h>

#include <exception>
#include <iostream>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: franka_error_recovery <robot-ip>\n";
    return 2;
  }
  try {
    franka::Robot robot(argv[1]);
    robot.automaticErrorRecovery();
    std::cout << "Franka automatic error recovery succeeded.\n";
    return 0;
  } catch (const franka::Exception& error) {
    std::cerr << "Franka recovery failed: " << error.what() << '\n';
  } catch (const std::exception& error) {
    std::cerr << "Recovery failed: " << error.what() << '\n';
  }
  return 1;
}
