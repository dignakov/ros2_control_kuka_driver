// Copyright 2020 ROS2-Control Development Team
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

#include "ros2_control_kuka_driver/kuka_system_position_only.hpp"

#include <thread>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;


namespace ros2_control_kuka_driver
{

return_type KukaSystemPositionOnlyHardware::configure(const hardware_interface::HardwareInfo &info)
{
	RCLCPP_INFO(rclcpp::get_logger("KukaSystemPositionOnlyHardware"),
	"configure()");

	if(configure_default(info) != return_type::OK){
		return return_type::ERROR;
	}

	hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
	hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

	for (const hardware_interface::ComponentInfo &joint : info_.joints){

		if(joint.command_interfaces.size() != 1){
			RCLCPP_FATAL(rclcpp::get_logger("KukaSystemPositionOnlyHardware"), "expecting exactly 1 command interface");
			return return_type::ERROR;
		}

		if(joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION){
			RCLCPP_FATAL(rclcpp::get_logger("KukaSystemPositionOnlyHardware"), "expecting only POSITION command interface");
			return return_type::ERROR;
		}

		if(joint.state_interfaces.size() != 1){
			RCLCPP_FATAL(rclcpp::get_logger("KukaSystemPositionOnlyHardware"), "expecting exactly 1 state interface");
			return return_type::ERROR;
		}

		if(joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION){
			RCLCPP_FATAL(rclcpp::get_logger("KukaSystemPositionOnlyHardware"), "expecting only POSITION state interface");
			return return_type::ERROR;
		}

	}



	//RSI
	in_buffer_.resize(1024);
	out_buffer_.resize(1024);
	remote_host_.resize(1024);
	remote_port_.resize(1024);

	rsi_initial_joint_positions_.resize(6, 0.0);
	rsi_joint_position_corrections_.resize(6, 0.0),
	ipoc_ = 0;

  	local_host_ = info_.hardware_parameters["listen_address"];
	local_port_ = stoi(info_.hardware_parameters["listen_port"]);

	RCLCPP_INFO(rclcpp::get_logger("KukaSystemPositionOnlyHardware"),
	"robot location: %s:%d", local_host_.c_str(), local_port_);

	//done
	status_ = hardware_interface::status::CONFIGURED;
	return return_type::OK;
}

std::vector<hardware_interface::StateInterface> KukaSystemPositionOnlyHardware::export_state_interfaces()
{
	RCLCPP_INFO(rclcpp::get_logger("KukaSystemPositionOnlyHardware"),
	"export_state_interfaces()");

	std::vector<hardware_interface::StateInterface> state_interfaces;
	for(size_t i=0; i<info_.joints.size(); i++){
		state_interfaces.emplace_back(
				hardware_interface::StateInterface(
					info_.joints[i].name,
					hardware_interface::HW_IF_POSITION,
					&hw_states_[i]));
	}
	return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> KukaSystemPositionOnlyHardware::export_command_interfaces()
{
	RCLCPP_INFO(rclcpp::get_logger("KukaSystemPositionOnlyHardware"),
	"export_command_interfaces()");

	std::vector<hardware_interface::CommandInterface> command_interfaces;
	for(size_t i=0; i<info_.joints.size(); i++){
		command_interfaces.emplace_back(
				hardware_interface::CommandInterface(
					info_.joints[i].name,
					hardware_interface::HW_IF_POSITION,
					&hw_commands_[i]));
	}
	return command_interfaces;
}

return_type KukaSystemPositionOnlyHardware::start()  // QUESTION: should this be in configure?
{
	RCLCPP_INFO(rclcpp::get_logger("KukaSystemPositionOnlyHardware"),
	"Starting ...please wait...");

	// Wait for connection from robot
	server_.reset(new UDPServer(local_host_, local_port_));

	RCLCPP_INFO(rclcpp::get_logger("KukaSystemPositionOnlyHardware"),
	"Connecting to robot . . .");

	int bytes = server_->recv(in_buffer_);

	RCLCPP_INFO(rclcpp::get_logger("KukaSystemPositionOnlyHardware"),
	"got some bytes");

	// Drop empty <rob> frame with RSI <= 2.3
	if(bytes < 100){
		bytes = server_->recv(in_buffer_);
	}
	if(bytes < 100){
		RCLCPP_FATAL(rclcpp::get_logger("KukaSystemPositionOnlyHardware"),
		"not enough data received");
		return return_type::ERROR;
	}


	rsi_state_ = kuka_rsi_hw_interface::RSIState(in_buffer_);

	for (size_t i = 0; i < hw_states_.size(); ++i){
		hw_states_[i] = rsi_state_.positions[i] * 3.14159/180.0;
		hw_commands_[i] = hw_states_[i];
		rsi_initial_joint_positions_[i] = rsi_state_.initial_positions[i];
	}
	ipoc_ = rsi_state_.ipoc;

	out_buffer_ = kuka_rsi_hw_interface::RSICommand(rsi_joint_position_corrections_, ipoc_).xml_doc;
	server_->send(out_buffer_);
	server_->set_timeout(1000); // Set receive timeout to 1 second

	RCLCPP_INFO(rclcpp::get_logger("KukaSystemPositionOnlyHardware"),
	"System Sucessfully started!");

	status_ = hardware_interface::status::STARTED;
	return return_type::OK;
}

return_type KukaSystemPositionOnlyHardware::stop()
{

	RCLCPP_INFO(rclcpp::get_logger("KukaSystemPositionOnlyHardware"),
	"Stopping ...please wait...");



	RCLCPP_INFO(rclcpp::get_logger("KukaSystemPositionOnlyHardware"),
	"System sucessfully stopped!");

	status_ = hardware_interface::status::STOPPED;
	return return_type::OK;
}

return_type KukaSystemPositionOnlyHardware::read()
{
	RCLCPP_INFO(rclcpp::get_logger("KukaSystemPositionOnlyHardware"), "read()");

	in_buffer_.resize(1024);
	if (server_->recv(in_buffer_) == 0){
		return return_type::ERROR;
	}

	rsi_state_ = kuka_rsi_hw_interface::RSIState(in_buffer_);

  	for (size_t i = 0; i < hw_states_.size(); i++) {
		hw_states_[i] = rsi_state_.positions[i] * 3.14159/180.0;
    	RCLCPP_INFO(rclcpp::get_logger("KukaSystemPositionOnlyHardware"), "Got state %.5f for joint %d!",hw_states_[i], i);
  	}
	ipoc_ = rsi_state_.ipoc;

	return return_type::OK;
}

return_type KukaSystemPositionOnlyHardware::write()
{
	RCLCPP_INFO(rclcpp::get_logger("KukaSystemPositionOnlyHardware"), "write()");

	out_buffer_.resize(1024);

  	for (size_t i = 0; i < hw_commands_.size(); i++) {
    	RCLCPP_INFO(rclcpp::get_logger("KukaSystemPositionOnlyHardware"), "Got command %.5f for joint %d!",hw_commands_[i], i);
		rsi_joint_position_corrections_[i] = (hw_commands_[i]*180.0/3.14159) - rsi_initial_joint_positions_[i];
  	}
	out_buffer_ = kuka_rsi_hw_interface::RSICommand(rsi_joint_position_corrections_, ipoc_).xml_doc;
	server_->send(out_buffer_);

	return return_type::OK;
}



} //namespace ros2_control_kuka_driver


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
	ros2_control_kuka_driver::KukaSystemPositionOnlyHardware,
	hardware_interface::SystemInterface
)

