#include "rclcpp/rclcpp.hpp"
#include "imu_serial_reader/ImuSerialReceiver.hpp"
#include "imu_serial_reader/stateMachine.hpp"
#include "imu_serial_reader/filter.hpp"
#include <chrono>
#include <thread>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImuSerialReceiver>("/dev/bno_imu", 460800);

  node->openPort();

  Filter acc_filter(static_cast<float>(node->getFilterAlphaAcc()));
  Filter gyro_filter(static_cast<float>(node->getFilterAlphaGyro()));
  Filter gravity_filter(static_cast<float>(node->getFilterAlphaGravity()));
  Filter quat_filter(static_cast<float>(node->getFilterAlphaQuat()));

  StateMachine state_machine(node.get(), &acc_filter, &gyro_filter, &gravity_filter, &quat_filter);
  state_machine.setState(StateMachine::State::READING);

  // 메인 루프
  while(rclcpp::ok())
  {
    state_machine.StateControl();
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  node->closePort();
  rclcpp::shutdown();
  return 0;
}
