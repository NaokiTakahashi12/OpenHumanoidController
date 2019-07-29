
/**
  *
  * @file CM730.hpp
  * @brief CM730 board communicator class
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include "../../Communicator/SerialFlowScheduler.hpp"
#include "../../Communicator/SerialController/Dynamixel.hpp"

namespace IO {
	namespace Device {
		namespace ControlBoard {
			class CM730 final {
				public :
					using SerialController = std::shared_ptr<Communicator::SerialController::Dynamixel>;
					CM730();

					void register_controller(SerialController &);

					Communicator::SerialFlowScheduler::Byte id() const;
					void id(const Communicator::SerialFlowScheduler::Byte &);

					void enable_power(const bool &flag);

					void ping();

				private :
					static constexpr Communicator::SerialFlowScheduler::Byte default_id = 0xc8;

					Communicator::SerialFlowScheduler::Byte current_id;

					Communicator::SerialFlowScheduler::SinglePacket create_switch_power_packet(const bool &);

					SerialController serial_controller; 
			};
		}
	}
}

