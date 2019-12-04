
/**
  *
  * @file B3MSC1170A.cpp
  * @authors Yasuo Hayashibara
  *			 Naoki Takahashi
  *
  **/

#include "B3MSC1170A.hpp"

#include <Tools/Byte.hpp>

#include "B3MSC1170AControlTable.hpp"
#include "../../../Communicator/Protocols/KondoB3M.hpp"

namespace IO {
	namespace Device {
		namespace Actuator {
			namespace ServoMotor {
				B3MSC1170A::B3MSC1170A(RobotStatus::InformationPtr &robot_status_information_ptr) : SerialServoMotor(robot_status_information_ptr) {
				}

				B3MSC1170A::B3MSC1170A(const ID &id, RobotStatus::InformationPtr &robot_status_information_ptr) : SerialServoMotor(id, robot_status_information_ptr) {
				}

				B3MSC1170A::~B3MSC1170A() {
				}

				std::string B3MSC1170A::get_key() {
					return "B3MSC1170A";
				}

				void B3MSC1170A::ping() {
					command_controller_access_assertion();

					command_controller->set_packet(
						create_ping_packet(id())
					);
				}

				void B3MSC1170A::enable_torque(const bool &flag) {
					command_controller_access_assertion();

					command_controller->set_packet(
						create_enable_torque_packet(id(), flag)
					);
				}

				void B3MSC1170A::write_gain(const WriteValue &p, const WriteValue &i, const WriteValue &d) {
					command_controller_access_assertion();

					write_gain_packet(id(), p, i, d);
				}

				void B3MSC1170A::write_angle(const float &degree) {
					command_controller_access_assertion();

					command_controller->set_packet(
						create_write_angle_packet(id(), degree)
					);
				}

				void B3MSC1170A::command_controller_access_assertion() {
					if(!command_controller) {
						throw std::runtime_error("Failed access to command_controller from IO::Device::Actuator::ServoMotor::B3MSC1170A");
					}
				}

				void B3MSC1170A::write_gain_packet(const ID &packet_id, const WriteValue &p, const WriteValue &i, const WriteValue &d) {
					command_controller->set_packet(
						create_write_p_gain_packet(packet_id, p)
					);
					command_controller->set_packet(
						create_write_i_gain_packet(packet_id, i)
					);
					command_controller->set_packet(
						create_write_d_gain_packet(packet_id, d)
					);
				}

				B3MSC1170A::SendPacket B3MSC1170A::create_ping_packet(const ID &) {
					SendPacket ret_packet;

					return ret_packet;
				}

				B3MSC1170A::SendPacket B3MSC1170A::create_enable_torque_packet(const ID &id, const bool &flag) {
					SendPacket value; 
					value += flag ? Communicator::Protocols::KondoB3M::enable :
								    Communicator::Protocols::KondoB3M::disable;

					const auto ret_packet = Communicator::Protocols::KondoB3M::create_write_packet(
						id,
						B3MSC1170AControlTable::servo_servo_mode,
						value
					);

					return ret_packet;
				}

				B3MSC1170A::SendPacket B3MSC1170A::create_write_angle_packet(const ID &id, const float &degree) {
					Communicator::Protocols::KondoB3M::Bytes write_degree_bytes;
					const auto degree_write_value = static_cast<short>(degree * 10);

					write_degree_bytes = static_cast<uint8_t>(Tools::Byte::low_byte(degree_write_value));
					write_degree_bytes += static_cast<uint8_t>(Tools::Byte::high_byte(degree_write_value));

					const auto ret_packet = Communicator::Protocols::KondoB3M::create_write_packet(
						id,
						B3MSC1170AControlTable::servo_desired_position,
						write_degree_bytes
					);

					return ret_packet;
				}

				B3MSC1170A::SendPacket B3MSC1170A::create_write_p_gain_packet(const ID &id, const WriteValue &gain) {
					SendPacket value;
					value += gain;

					const auto ret_packet = Communicator::Protocols::KondoB3M::create_write_packet(
						id,
						B3MSC1170AControlTable::control_kp0,
						value
					);
					
					return ret_packet;
				}

				B3MSC1170A::SendPacket B3MSC1170A::create_write_i_gain_packet(const ID &, const WriteValue &) {
					SendPacket ret_packet;
					
					return ret_packet;
				}

				B3MSC1170A::SendPacket B3MSC1170A::create_write_d_gain_packet(const ID &, const WriteValue &) {
					SendPacket ret_packet;
					
					return ret_packet;
				}
			}
		}
	}
}

