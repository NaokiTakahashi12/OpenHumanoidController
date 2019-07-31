
/**
  *
  * @file SerialControlBoard.hpp
  * @brief Control board utile class
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include "SerialControlBoardBase.hpp"

namespace IO {
	namespace Device {
		namespace ControlBoard {
			template <class CONTROLLER>
			class SerialControlBoard : public SerialControlBoardBase {
				protected :
					using CommandControllerPtr = std::shared_ptr<CONTROLLER>;

					CommandControllerPtr command_controller;

				public :
					SerialControlBoard(RobotStatus::InformationPtr &);
					SerialControlBoard(const ID &, RobotStatus::InformationPtr &);

					void register_controller(CommandControllerPtr &);

					virtual void ping(),
								 enable_power(const bool &flag);
			};
		}
	}
}

