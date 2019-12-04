
/**
  *
  * @file SerialServoMotor.cpp
  * @author Naoki Takahashi
  *
  **/

#include "SerialServoMotor.hpp"

#include <stdexcept>

namespace IO {
	namespace Device {
		namespace Actuator {
			namespace ServoMotor {
				SerialServoMotor::SerialServoMotor(RobotStatus::InformationPtr &robot_status_information_ptr) {
					this->robo_info = robot_status_information_ptr;
				}

				SerialServoMotor::SerialServoMotor(const ID &new_id, RobotStatus::InformationPtr &robot_status_information_ptr) : SerialServoMotor(robot_status_information_ptr) {
					id(new_id);
				}

				SerialServoMotor::~SerialServoMotor() {
				}

				void SerialServoMotor::ping() {
					throw std::logic_error("Unoverride from IO::Device::Actuator::ServoMotor::SerialServoMotor");
				}

				void SerialServoMotor::enable_torque(const bool &) {
					throw std::logic_error("Unoverride from IO::Device::Actuator::ServoMotor::SerialServoMotor");
				}

				void SerialServoMotor::write_gain(const WriteValue &, const WriteValue &, const WriteValue &) {
					throw std::logic_error("Unoverride from IO::Device::Actuator::ServoMotor::SerialServoMotor");
				}

				void SerialServoMotor::write_angle(const float &) {
					throw std::logic_error("Unoverride from IO::Device::Actuator::ServoMotor::SerialServoMotor");
				}

				SerialServoMotor::ID &SerialServoMotor::id() const {
					if(!device_id) {
						throw std::runtime_error("Failed access device_id from IO::Device::Actuator::ServoMotor::SerialServoMotor");
					}

					return *device_id;
				}

				void SerialServoMotor::id(const ID &new_id) {
					if(!device_id) {
						device_id = std::make_unique<ID>();
					}

					*device_id = new_id;
				}
			}
		}
	}
}

