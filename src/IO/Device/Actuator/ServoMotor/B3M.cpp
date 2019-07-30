/**
  *
  * @file B3M.cpp
  * @auther Yasuo Hayashibara
  *
  **/

#include "B3M.hpp"

#include <stdexcept>

#include <Tools/Byte.hpp>

#include "B3MControlTable.hpp"
#include "../../../Communicator/Protocols/KondoB3M.hpp"

namespace IO {
	namespace Device {
		namespace Actuator {
			namespace ServoMotor {
				B3M::B3M() : SerialServoMotorBase() {
				}

				void B3M::register_controller(std::shared_ptr<SerialController> &serial_controller_ptr) {
					serial_controller = serial_controller_ptr;
				}

				void B3M::enable_torque(const bool &flag, const ID &id) {
					if(!serial_controller) {
						throw std::runtime_error("Can not access serial controller");
					}
					serial_controller->packet(create_switch_torque_packet(flag, id));
				}

				void B3M::write_p_gain(const unsigned short &gain, const ID &id) {
					if(!serial_controller) {
						throw std::runtime_error("Can not access serial controller");
					}
					serial_controller->packet(create_write_p_gain_packet(gain, id));
				}

				void B3M::write_angle(const float &degree, const ID &id) {
					if(!serial_controller) {
						throw std::runtime_error("Can not access serial controller");
					}
					serial_controller->packet(create_angle_write_packet(degree, id));
				}

				void B3M::ping(const ID &id) {
				}

				B3M::SendPacket B3M::create_switch_torque_packet(const bool &flag, const ID &id) {
					std::string value;
                    value += flag ? Communicator::Protocols::KondoB3M::enable :
									Communicator::Protocols::KondoB3M::disable;
					auto packet = Communicator::Protocols::KondoB3M::create_write_packet(
							id,
							B3MControlTable::servo_servo_mode,
							value
					);
					return packet;
				}

				B3M::SendPacket B3M::create_write_p_gain_packet(const unsigned short &gain, const ID &id) {
					std::string value;
                    value += gain;
                    auto packet = Communicator::Protocols::KondoB3M::create_write_packet(
							id,
							B3MControlTable::control_kp0,
							value
					);
					return packet;
				}

				B3M::SendPacket B3M::create_angle_write_packet(const float &degree, const ID &id) {
					Communicator::Protocols::KondoB3M::Bytes write_degree_bytes;
					short degree_write_value = degree * 10;

					write_degree_bytes = static_cast<uint8_t>(Tools::Byte::low_byte(degree_write_value));
					write_degree_bytes += static_cast<uint8_t>(Tools::Byte::high_byte(degree_write_value));

					auto packet = Communicator::Protocols::KondoB3M::create_write_packet(
							id,
							B3MControlTable::servo_desired_position,
							write_degree_bytes
					);
					return packet;
				}
			}
		}
	}
}
