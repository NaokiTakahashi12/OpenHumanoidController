
/**
  *
  * @file SerialServoMotor.cpp
  * @author Naoki Takahashi
  *
  **/

#include "SerialServoMotor.hpp"

#include <stdexcept>

#include "../../../Communicator/SerialController/Dynamixel.hpp"

namespace IO {
	namespace Device {
		namespace Actuator {
			namespace ServoMotor {
				template <class CONTROLLER>
				SerialServoMotor<CONTROLLER>::SerialServoMotor(RobotStatus::InformationPtr &robot_status_information_ptr) : SerialServoMotorBase(robot_status_information_ptr) {
				}

				template <class CONTROLLER>
				SerialServoMotor<CONTROLLER>::SerialServoMotor(const ID &new_id, RobotStatus::InformationPtr &robot_status_information_ptr) : SerialServoMotorBase(new_id, robot_status_information_ptr) {
				}

				template <class CONTROLLER>
				void SerialServoMotor<CONTROLLER>::register_controller(CommandControllerPtr &command_controller) {
					if(!this->command_controller) {
						this->command_controller = command_controller;
					}
					else {
						throw std::runtime_error("Can not regist command controller from IO::Device::Actuator::ServoMotor::SerialServoMotor");
					}
				}

				template <class CONTROLLER>
				void SerialServoMotor<CONTROLLER>::ping() {
					throw std::logic_error("Unoverride from IO::Device::Actuator::ServoMotor::SerialServoMotor");
				}

				template <class CONTROLLER>
				void SerialServoMotor<CONTROLLER>::enable_torque(const bool &) {
					throw std::logic_error("Unoverride from IO::Device::Actuator::ServoMotor::SerialServoMotor");
				}

				template <class CONTROLLER>
				void SerialServoMotor<CONTROLLER>::write_gain(const WriteValue &, const WriteValue &, const WriteValue &) {
					throw std::logic_error("Unoverride from IO::Device::Actuator::ServoMotor::SerialServoMotor");
				}

				template <class CONTROLLER>
				void SerialServoMotor<CONTROLLER>::write_angle(const float &) {
					throw std::logic_error("Unoverride from IO::Device::Actuator::ServoMotor::SerialServoMotor");
				}

				template class SerialServoMotor<Communicator::SerialController::Dynamixel>;
			}
		}
	}
}

