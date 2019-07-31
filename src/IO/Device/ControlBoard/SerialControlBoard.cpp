
/**
  *
  * @file SerialControlBoard.cpp
  * @author Naoki Takahashi
  *
  **/

#include "SerialControlBoard.hpp"

#include "../../Communicator/SerialController/Dynamixel.hpp"

namespace IO {
	namespace Device {
		namespace ControlBoard {
			template <class CONTROLLER>
			SerialControlBoard<CONTROLLER>::SerialControlBoard(RobotStatus::InformationPtr &robot_status_information_ptr) : SerialControlBoardBase(robot_status_information_ptr) {
			}

			template <class CONTROLLER>
			SerialControlBoard<CONTROLLER>::SerialControlBoard(const ID &new_id, RobotStatus::InformationPtr &robot_status_information_ptr) : SerialControlBoardBase(new_id, robot_status_information_ptr) {
			}

			template <class CONTROLLER>
			void SerialControlBoard<CONTROLLER>::register_controller(CommandControllerPtr &command_controller) {
				if(!this->command_controller) {
					this->command_controller = command_controller;
				}
				else {
					throw std::runtime_error("Can not regist command controller from IO::Device::ControlBoard::SerialControlBoard");
				}
			}

			template <class CONTROLLER>
			void SerialControlBoard<CONTROLLER>::ping() {
				throw std::logic_error("Unoverride from IO::Device::ControlBoard::SerialControlBoard");
			}

			template <class CONTROLLER>
			void SerialControlBoard<CONTROLLER>::enable_power(const bool &) {
				throw std::logic_error("Unoverride from IO::Device::ControlBoard::SerialControlBoard");
			}

			template class SerialControlBoard<Communicator::SerialController::Dynamixel>;
		}
	}
}

