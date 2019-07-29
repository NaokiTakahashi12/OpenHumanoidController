
/**
  *
  * @file MX28.cpp
  * @auther Naoki Takahashi
  *
  **/

#include "MX28.hpp"

#include <stdexcept>

#include <Tools/Byte.hpp>

#include "MX28ControlTable.hpp"
#include "../../../Communicator/Protocols/DynamixelVersion1.hpp"

namespace IO {
	namespace Device {
		namespace Actuator {
			namespace ServoMotor {
				MX28::MX28() : SerialServoMotorBase() {
				}

				void MX28::register_controller(std::shared_ptr<SerialController> &serial_controller_ptr) {
					serial_controller = serial_controller_ptr;
				}

				void MX28::enable_torque(const bool &flag, const ID &id) {
					if(!serial_controller) {
						throw std::runtime_error("Can not access serial controller");
					}
					serial_controller->packet(create_switch_torque_packet(flag, id));
				}

				void MX28::write_p_gain(const unsigned short &gain, const ID &id) {
					if(!serial_controller) {
						throw std::runtime_error("Can not access serial controller");
					}
					serial_controller->packet(create_write_p_gain_packet(gain, id));
				}

				void MX28::write_angle(const float &degree, const ID &id) {
					if(!serial_controller) {
						throw std::runtime_error("Can not access serial controller");
					}
					serial_controller->packet(create_angle_write_packet(degree, id));
				}

				void MX28::ping(const ID &id) {
					if(!serial_controller) {
						throw std::runtime_error("Can not access serial controller");
					}
					serial_controller->packet(Communicator::Protocols::DynamixelVersion1::create_ping_packet(id));
				}

				MX28::SendPacket MX28::create_switch_torque_packet(const bool &flag, const ID &id) {
					auto value = flag ? Communicator::Protocols::DynamixelVersion1::enable :
										Communicator::Protocols::DynamixelVersion1::disable;
					auto packet = Communicator::Protocols::DynamixelVersion1::create_write_packet(
							id,
							MX28ControlTable::enable_torque,
							value
					);
					return packet;
				}

				MX28::SendPacket MX28::create_write_p_gain_packet(const unsigned short &gain, const ID &id) {
					auto packet = Communicator::Protocols::DynamixelVersion1::create_write_packet(
							id,
							MX28ControlTable::proportional_gain,
							static_cast<Communicator::Protocols::DynamixelVersion1::Byte>(gain)
					);
					return packet;
				}

				MX28::SendPacket MX28::create_angle_write_packet(const float &degree, const ID &id) {
					Communicator::Protocols::DynamixelVersion1::Bytes write_degree_bytes;
					short degree_write_value = degree * 10;

					write_degree_bytes = static_cast<uint8_t>(Tools::Byte::low_byte(degree_write_value));
					write_degree_bytes += static_cast<uint8_t>(Tools::Byte::high_byte(degree_write_value));

					auto packet = Communicator::Protocols::DynamixelVersion1::create_write_packet(
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

