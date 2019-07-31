
/**
  *
  * @file MX28.cpp
  * @auther Naoki Takahashi
  *
  **/

#include "MX28.hpp"

#include <Tools/Byte.hpp>

#include "MX28ControlTable.hpp"
#include "../../../Communicator/Protocols/DynamixelVersion1.hpp"

namespace IO {
	namespace Device {
		namespace Actuator {
			namespace ServoMotor {
				MX28::MX28(RobotStatus::InformationPtr &robot_status_information_ptr) : SerialServoMotor(robot_status_information_ptr) {
				}

				MX28::MX28(const ID &new_id, RobotStatus::InformationPtr &robot_status_information_ptr) : SerialServoMotor(new_id, robot_status_information_ptr) {
				}

				void MX28::ping() {
					if(!command_controller) {
						throw std::runtime_error("Can not access serial controller from IO::Device::Actuator::ServoMotor::MX28");
					}
					command_controller->set_packet(Communicator::Protocols::DynamixelVersion1::create_ping_packet(id()));
				}

				void MX28::enable_torque(const bool &flag) {
					if(!command_controller) {
						throw std::runtime_error("Can not access serial controller from IO::Device::Actuator::ServoMotor::MX28");
					}
					command_controller->set_packet(create_switch_torque_packet(flag, id()));
				}

				void MX28::write_gain(const WriteValue &p, const WriteValue &i, const WriteValue &d) {
					write_p_gain(p);
					write_i_gain(i);
					write_d_gain(d);
				}

				void MX28::write_angle(const float &degree) {
					if(!command_controller) {
						throw std::runtime_error("Can not access serial controller from IO::Device::Actuator::ServoMotor::MX28");
					}
					command_controller->set_packet(create_angle_write_packet(degree, id()));
				}

				void MX28::write_p_gain(const WriteValue &gain) {
					if(!command_controller) {
						throw std::runtime_error("Can not access serial controller from IO::Device::Actuator::ServoMotor::MX28");
					}
					command_controller->set_packet(create_write_p_gain_packet(gain, id()));
				}

				void MX28::write_i_gain(const WriteValue &gain) {
					if(!command_controller) {
						throw std::runtime_error("Can not access serial controller from IO::Device::Actuator::ServoMotor::MX28");
					}
					command_controller->set_packet(create_write_i_gain_packet(gain, id()));
				}

				void MX28::write_d_gain(const WriteValue &gain) {
					if(!command_controller) {
						throw std::runtime_error("Can not access serial controller from IO::Device::Actuator::ServoMotor::MX28");
					}
					command_controller->set_packet(create_write_d_gain_packet(gain, id()));
				}

				MX28::SendPacket MX28::create_switch_torque_packet(const bool &flag, const ID &id) const {
					const auto value = flag ? Communicator::Protocols::DynamixelVersion1::enable :
										Communicator::Protocols::DynamixelVersion1::disable;
					const auto packet = Communicator::Protocols::DynamixelVersion1::create_write_packet(
							id,
							MX28ControlTable::enable_torque,
							value
					);
					return packet;
				}

				MX28::SendPacket MX28::create_write_p_gain_packet(const WriteValue &gain, const ID &id) const {
					const auto packet = Communicator::Protocols::DynamixelVersion1::create_write_packet(
							id,
							MX28ControlTable::proportional_gain,
							static_cast<Communicator::Protocols::DynamixelVersion1::Byte>(gain)
					);
					return packet;
				}

				MX28::SendPacket MX28::create_write_i_gain_packet(const WriteValue &gain, const ID &id) const {
					const auto packet = Communicator::Protocols::DynamixelVersion1::create_write_packet(
							id,
							MX28ControlTable::integral_gain,
							static_cast<Communicator::Protocols::DynamixelVersion1::Byte>(gain)
					);
					return packet;
				}

				MX28::SendPacket MX28::create_write_d_gain_packet(const WriteValue &gain, const ID &id) const {
					const auto packet = Communicator::Protocols::DynamixelVersion1::create_write_packet(
							id,
							MX28ControlTable::derivative_gain,
							static_cast<Communicator::Protocols::DynamixelVersion1::Byte>(gain)
					);
					return packet;
				}

				MX28::SendPacket MX28::create_angle_write_packet(const float &degree, const ID &id) const {
					Communicator::Protocols::DynamixelVersion1::Bytes write_degree_bytes;
					const short degree_write_value = degree * 10;

					write_degree_bytes = static_cast<uint8_t>(Tools::Byte::low_byte(degree_write_value));
					write_degree_bytes += static_cast<uint8_t>(Tools::Byte::high_byte(degree_write_value));

					const auto packet = Communicator::Protocols::DynamixelVersion1::create_write_packet(
							id,
							MX28ControlTable::goal_position_low,
							write_degree_bytes
					);
					return packet;
				}
			}
		}
	}
}

