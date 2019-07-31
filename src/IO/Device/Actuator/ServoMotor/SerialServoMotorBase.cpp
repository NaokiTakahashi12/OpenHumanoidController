
/**
  *
  * @file SerialServoMotorBase.cpp
  * @author Naoki Takahashi
  *
  **/

#include "SerialServoMotorBase.hpp"

namespace IO {
	namespace Device {
		namespace Actuator {
			namespace ServoMotor {
				SerialServoMotorBase::SerialServoMotorBase(RobotStatus::InformationPtr &robot_status_information_ptr) {
					device_id = std::make_unique<ID>();
					robo_info = robot_status_information_ptr;
				}

				SerialServoMotorBase::SerialServoMotorBase(const ID &new_id, RobotStatus::InformationPtr &robot_status_information_ptr) : SerialServoMotorBase(robot_status_information_ptr) {
					id(new_id);
				}

				SerialServoMotorBase::SerialServoMotorBase(const SerialServoMotorBase &serial_servo_motor_base) {
					if(this != &serial_servo_motor_base) {
						this->device_id = std::make_unique<ID>(*serial_servo_motor_base.device_id);
						this->robo_info = serial_servo_motor_base.robo_info;
					}
				}

				SerialServoMotorBase::~SerialServoMotorBase() {
				}

				SerialServoMotorBase::ID &SerialServoMotorBase::id() const {
					if(!device_id) {
						throw std::runtime_error("Can not return device id from IO::Device::Actuator::ServoMotor:SerialServoMotorBase");
					}
					return *device_id;
				}

				void SerialServoMotorBase::id(const ID &new_id) const {
					if(!device_id) {
						throw std::runtime_error("Can not return device id from IO::Device::Actuator::ServoMotor:SerialServoMotorBase");
					}
					*device_id = new_id;
				}
			}
		}
	}
}

