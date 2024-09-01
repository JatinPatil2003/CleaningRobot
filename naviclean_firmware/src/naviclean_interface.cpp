// Copyright (c) 2024 Jatin Patil
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>

#include "naviclean_firmware/naviclean_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"


namespace naviclean_firmware
{
NavicleanInterface::NavicleanInterface()
{
}

NavicleanInterface::~NavicleanInterface()
{
  if (esp_.IsOpen()) {
    try {
      esp_.Close();
    } catch (...) {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("NavicleanInterface"),
        "Something went wrong while closing connection with port " << port_);
    }
  }
}

hardware_interface::CallbackReturn NavicleanInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    port_ = info_.hardware_parameters.at("port");
  } catch (const std::out_of_range & e) {
    RCLCPP_FATAL(rclcpp::get_logger("NavicleanInterface"), "No Serial Port provided! Aborting");
    return CallbackReturn::FAILURE;
  }

  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> NavicleanInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> NavicleanInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn NavicleanInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("NavicleanInterface"), "Activating ...please wait...");

  // set some default values
  for (auto i = 0u; i < hw_positions_.size(); i++) {
    if (std::isnan(hw_positions_[i])) {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  if (esp_.IsOpen()) {
    try {
      esp_.Close();
    } catch (...) {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("NavicleanInterface"),
        "Something went wrong while closing connection with port " << port_);
    }
  }

  try {
    esp_.Open(port_);
    esp_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
  } catch (...) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("NavicleanInterface"),
                        "Something went wrong while interacting with port " << port_);
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("NavicleanInterface"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn NavicleanInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("NavicleanInterface"), "Deactivating ...please wait...");

  if (esp_.IsOpen()) {
    try {
      esp_.Close();
    } catch (...) {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("NavicleanInterface"),
        "Something went wrong while closing connection with port " << port_);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("NavicleanInterface"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type NavicleanInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::string message;
  esp_.ReadLine(message);
  
  size_t commaIndex = message.find(',');
  if (commaIndex != std::string::npos) {
    std::string leftPos = message.substr(0, commaIndex);
    std::string rightPos = message.substr(commaIndex + 1);

    // RCLCPP_INFO(rclcpp::get_logger("NavicleanInterface"), 
    //   "Read - %s, %s", leftPos.c_str(), rightPos.c_str());

    try {
      hw_positions_[1] = std::stof(leftPos) * -1;
      hw_positions_[0] = std::stof(rightPos);
    } catch(const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("NavicleanInterface"), 
        "Invalid argument for conversion: %s", e.what());
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type naviclean_firmware::NavicleanInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::stringstream ss;
  auto leftCmd = hw_commands_[1] * -1;
  auto rightCmd = hw_commands_[0];

  // RCLCPP_INFO(rclcpp::get_logger("NavicleanInterface"), 
  //   "Write - %f, %f", leftCmd, rightCmd);

  ss << leftCmd << "," << rightCmd << "\n";
  try {
    esp_.Write(ss.str());
  } catch (...) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("NavicleanInterface"),
      "Something went wrong while sending the message " << ss.str() << " to the port " << port_);
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace naviclean_firmware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(naviclean_firmware::NavicleanInterface,
  hardware_interface::SystemInterface)
