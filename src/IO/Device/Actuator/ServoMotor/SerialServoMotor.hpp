
/**
  *
  * @file SerialServoMotor.hpp
  * @brief Servo motor class
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include <RobotStatus/Information.hpp>

#include "SerialServoMotorBase.hpp"

namespace IO {
	namespace Device {
		namespace Actuator {
			namespace ServoMotor {
				template <class CONTROLLER, class DEVICE>
				class SerialServoMotor final {
					private :
						using ServoMotorDevicePtr = std::unique_ptr<DEVICE>;
						using CommandControllerPtr = std::shared_ptr<CONTROLLER>;

						ServoMotorDevicePtr servo_motor_device;
						CommandControllerPtr command_controller;

					public :
						SerialServoMotor();
						SerialServoMotor(RobotStatus::InformationPtr &);

						void register_controller(CommandControllerPtr &);

						void enable_torque(const bool &flag, const SerialServoMotorBase::ID &id),
							 ping(const SerialServoMotorBase::ID &id),
							 write_gain(const unsigned short &p, const unsigned short &i, const unsigned short &d, const SerialServoMotorBase::ID &id),
							 write_angle(const float &degree, const SerialServoMotorBase::ID &id);

						CommandControllerPtr get_controller_ptr();

				};
			}
		}
	}
}

