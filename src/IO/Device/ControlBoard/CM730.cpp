
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
			CM730::CM730() {
				current_id = default_id;
			}

			void CM730::register_controller(SerialController &new_serial_controller) {
				serial_controller = new_serial_controller;
			}

			Communicator::SerialFlowScheduler::Byte CM730::id() const {
				return current_id;
			}

			void CM730::id(const Communicator::SerialFlowScheduler::Byte &new_id) {
				current_id = new_id;
			}

			void CM730::enable_power(const bool &flag) {
				serial_controller->packet(create_switch_power_packet(flag));
			}

			void CM730::ping() {
				serial_controller->packet(Communicator::Protocols::DynamixelVersion1::create_ping_packet(current_id));
			}

			Communicator::SerialFlowScheduler::SinglePacket CM730::create_switch_power_packet(const bool &flag) {
				auto value = flag ? Communicator::Protocols::DynamixelVersion1::enable :
									Communicator::Protocols::DynamixelVersion1::disable;
				auto packet = Communicator::Protocols::DynamixelVersion1::create_write_packet(
						current_id,
						CM730ControlTable::dynamixel_power,
						value
				);
				return packet;
			}
		}
	}
}

