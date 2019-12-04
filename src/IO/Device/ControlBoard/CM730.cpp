
/**
  *
  * @file CM730.cpp
  * @author Naoki Takahashi
  *
  **/

#include "CM730.hpp"

#include "CM730ControlTable.hpp"

namespace IO {
	namespace Device {
		namespace ControlBoard {
			CM730::CM730(RobotStatus::InformationPtr &robot_status_information_ptr) : SerialControlBoard(robot_status_information_ptr) {
				id(default_id);
			}

			CM730::CM730(const ID &new_id, RobotStatus::InformationPtr &robot_status_information_ptr) : SerialControlBoard(new_id, robot_status_information_ptr) {
			}

			CM730::~CM730() {
			}

			std::string CM730::get_key() {
				return "CM730";
			}

			void CM730::enable_power(const bool &flag) {
				command_controller->set_packet(create_switch_power_packet(flag));

				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			}

			void CM730::ping() {
				command_controller->set_packet(Communicator::Protocols::DynamixelVersion1::create_ping_packet(id()));
			}

			Communicator::SerialFlowScheduler::SinglePacket CM730::create_switch_power_packet(const bool &flag) {
				const auto value = flag ? Communicator::Protocols::DynamixelVersion1::enable : Communicator::Protocols::DynamixelVersion1::disable;
				const auto packet = Communicator::Protocols::DynamixelVersion1::create_write_packet(
						id(),
						CM730ControlTable::dynamixel_power,
						value
				);

				return packet;
			}
		}
	}
}

