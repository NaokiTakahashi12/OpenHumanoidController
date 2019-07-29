
/**
  *
  * @file SerialControlBoard.cpp
  * @author Naoki Takahashi
  *
  **/

#include "SerialControlBoard.hpp"

#include "../../Communicator/SerialController/Dynamixel.hpp"
#include "CM730.hpp"

namespace IO {
	namespace Device {
		namespace ControlBoard {
			template <class CONTROLLER, class DEVICE>
			SerialControlBoard<CONTROLLER, DEVICE>::SerialControlBoard() {
				control_board_device = std::make_unique<DEVICE>();
			}

			template <class CONTROLLER, class DEVICE>
			SerialControlBoard<CONTROLLER, DEVICE>::SerialControlBoard(RobotStatus::InformationPtr &robot_status_information_ptr) {
				SerialControlBoard();
				command_controller = std::make_unique<CONTROLLER>(robot_status_information_ptr);
			}

			template <class CONTROLLER, class DEVICE>
			void SerialControlBoard<CONTROLLER, DEVICE>::register_controller(CommandControllerPtr &new_command_controller) {
				command_controller = new_command_controller;
				control_board_device->register_controller(command_controller);
			}

			template <class CONTROLLER, class DEVICE>
			void SerialControlBoard<CONTROLLER, DEVICE>::enable_power(const bool &flag) {
				control_board_device->enable_power(flag);
			}

			template <class CONTROLLER, class DEVICE>
			void SerialControlBoard<CONTROLLER, DEVICE>::ping() {
				control_board_device->ping();
			}

			template <class CONTROLLER, class DEVICE>
			void SerialControlBoard<CONTROLLER, DEVICE>::change_id(const Communicator::SerialFlowScheduler::Byte &id) {
				control_board_device->id(id);
			}

			template <class CONTROLLER, class DEVICE>
			Communicator::SerialFlowScheduler::Byte SerialControlBoard<CONTROLLER, DEVICE>::get_id() {
				return control_board_device->id();
			}

			template <class CONTROLLER, class DEVICE>
			typename SerialControlBoard<CONTROLLER, DEVICE>::CommandControllerPtr SerialControlBoard<CONTROLLER, DEVICE>::get_controller_ptr() {
				return command_controller;
			}

			template class SerialControlBoard<Communicator::SerialController::Dynamixel, CM730>;
		}
	}
}

