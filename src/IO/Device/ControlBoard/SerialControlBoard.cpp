
/**
  *
  * @file SerialControlBoard.cpp
  * @author Naoki Takahashi
  *
  **/

#include "SerialControlBoard.hpp"

namespace IO {
	namespace Device {
		namespace ControlBoard {
			SerialControlBoard::SerialControlBoard(RobotStatus::InformationPtr &robot_status_information_ptr) {
				this->robo_info = robot_status_information_ptr;
			}

			SerialControlBoard::SerialControlBoard(const ID &new_id, RobotStatus::InformationPtr &robot_status_information_ptr) : SerialControlBoard(robot_status_information_ptr) {
				id(new_id);
			}

			SerialControlBoard::~SerialControlBoard() {
			}

			void SerialControlBoard::ping() {
				throw std::logic_error("Unoverride from IO::Device::ControlBoard::SerialControlBoard");
			}

			void SerialControlBoard::enable_power(const bool &) {
				throw std::logic_error("Unoverride from IO::Device::ControlBoard::SerialControlBoard");
			}

			SerialControlBoard::ID &SerialControlBoard::id() const {
				if(!device_id) {
					throw std::runtime_error("Failed access to device_id from IO::Device::ControlBoard::SerialControlBoard");
				}

				return *device_id;
			}

			void SerialControlBoard::id(const ID &new_id) {
				if(!device_id) {
					device_id = std::make_unique<ID>();
				}

				*device_id = new_id;
			}
		}
	}
}

