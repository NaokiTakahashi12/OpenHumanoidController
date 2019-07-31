
/**
  *
  * @file SerialControlBoardBase.cpp
  * @author Naoki Takahashi
  *
  **/

#include "SerialControlBoardBase.hpp"

#include <stdexcept>

namespace IO {
	namespace Device {
		namespace ControlBoard {
			SerialControlBoardBase::SerialControlBoardBase(RobotStatus::InformationPtr &robot_status_information_ptr) {
				device_id = std::make_unique<ID>();
				robo_info = robot_status_information_ptr;
			}

			SerialControlBoardBase::SerialControlBoardBase(const ID &new_id, RobotStatus::InformationPtr &robot_status_information_ptr) : SerialControlBoardBase(robot_status_information_ptr) {
				id(new_id);
			}

			SerialControlBoardBase::~SerialControlBoardBase() {
			}

			SerialControlBoardBase::ID &SerialControlBoardBase::id() const {
				if(!device_id) {
					throw std::runtime_error("Can not return device id from IO::Device::ControlBoard::SerialControlBoardBase");
				}
				return *device_id;
			}

			void SerialControlBoardBase::id(const ID &new_id) const {
				if(!device_id) {
					throw std::runtime_error("Can not reset device id from IO::Device::ControlBoard::SerialControlBoardBase");
				}
				*device_id = new_id;
			}
		}
	}
}

