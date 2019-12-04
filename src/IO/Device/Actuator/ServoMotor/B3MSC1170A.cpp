
/**
  *
  * @file B3MSC1170A.cpp
  * @author Napki Takahashi
  *
  **/

#include "B3MSC1170A.hpp"

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

				B3MSC1170A::SendPacket B3MSC1170A::create_enable_torque_packet(const ID &, const bool &) {
					SendPacket ret_packet;

					return ret_packet;
				}

				B3MSC1170A::SendPacket B3MSC1170A::create_write_angle_packet(const ID &, const float &) {
					SendPacket ret_packet;

					return ret_packet;
				}

				B3MSC1170A::SendPacket B3MSC1170A::create_write_p_gain_packet(const ID &, const WriteValue &) {
					SendPacket ret_packet;
					
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

