
/**
  *
  * @file SerialControlBoard.hpp
  * @brief Control board utile class
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include <RobotStatus/Information.hpp>

#include "../../Communicator/SerialFlowScheduler.hpp"

namespace IO {
	namespace Device {
		namespace ControlBoard {
			template <class CONTROLLER, class DEVICE>
			class SerialControlBoard final {
				private :
					using ControlBoardDevicePtr = std::unique_ptr<DEVICE>;
					using CommandControllerPtr = std::shared_ptr<CONTROLLER>;

					ControlBoardDevicePtr control_board_device;
					CommandControllerPtr command_controller;

				public :
					SerialControlBoard();
					SerialControlBoard(RobotStatus::InformationPtr &);

					void register_controller(CommandControllerPtr &);

					void enable_power(const bool &flag),
						 ping(),
						 change_id(const Communicator::SerialFlowScheduler::Byte &id);

					Communicator::SerialFlowScheduler::Byte get_id();

					CommandControllerPtr get_controller_ptr();
			};
		}
	}
}

