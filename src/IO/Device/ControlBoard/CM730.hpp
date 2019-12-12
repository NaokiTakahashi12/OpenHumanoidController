
/**
  *
  * @file CM730.hpp
  * @brief CM730 board communicator class
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include "SerialControlBoard.hpp"

#include "../../Communicator/SerialController/Dynamixel.hpp"

namespace IO {
	namespace Device {
		namespace ControlBoard {
			class CM730 final : public SerialControlBoard {
				public :
					CM730(RobotStatus::InformationPtr &);
					CM730(const ID &, RobotStatus::InformationPtr &);

					~CM730();

					static std::string get_key();

					void enable_power(const bool &flag) override,
					 	 ping() override;

				private :
					static constexpr ID default_id = 0xc8;

					SendPacket create_switch_power_packet(const bool &);
			};
		}
	}
}

