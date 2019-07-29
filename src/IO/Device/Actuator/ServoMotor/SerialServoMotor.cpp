
/**
  *
  * @file SerialServoMotor.cpp
  * @author Naoki Takahashi
  *
  **/

#include "SerialServoMotor.hpp"

#include "../../../Communicator/SerialController/Dynamixel.hpp"
#include "MX28.hpp"

namespace IO {
	namespace Device {
		namespace Actuator {
			namespace ServoMotor {
				template <class CONTROLLER, class DEVICE>
				SerialServoMotor<CONTROLLER, DEVICE>::SerialServoMotor() {
					servo_motor_device = std::make_unique<DEVICE>();
				}

				template <class CONTROLLER, class DEVICE>
				SerialServoMotor<CONTROLLER, DEVICE>::SerialServoMotor(RobotStatus::InformationPtr &robot_status_information_ptr) {
					SerialServoMotor();
					command_controller = std::make_unique<CONTROLLER>(robot_status_information_ptr);
				}

				template <class CONTROLLER, class DEVICE>
				void SerialServoMotor<CONTROLLER, DEVICE>::register_controller(CommandControllerPtr &controller) {
					command_controller = controller;
					servo_motor_device->register_controller(controller);
				}

				template <class CONTROLLER, class DEVICE>
				void SerialServoMotor<CONTROLLER, DEVICE>::enable_torque(const bool &flag, const SerialServoMotorBase::ID &id) {
					servo_motor_device->enable_torque(flag, id);
				}

				template <class CONTROLLER, class DEVICE>
				void SerialServoMotor<CONTROLLER, DEVICE>::ping(const SerialServoMotorBase::ID &id) {
					servo_motor_device->ping(id);
				}

				template <class CONTROLLER, class DEVICE>
				void SerialServoMotor<CONTROLLER, DEVICE>::write_gain(const unsigned short &p, const unsigned short &, const unsigned short &, const SerialServoMotorBase::ID &id) {
					servo_motor_device->write_p_gain(p, id);
				}

				template <class CONTROLLER, class DEVICE>
				void SerialServoMotor<CONTROLLER, DEVICE>::write_angle(const float &degree, const SerialServoMotorBase::ID &id) {
					servo_motor_device->write_angle(degree, id);
				}

				template <class CONTROLLER, class DEVICE>
				typename SerialServoMotor<CONTROLLER, DEVICE>::CommandControllerPtr SerialServoMotor<CONTROLLER, DEVICE>::get_controller_ptr() {
					return command_controller;
				}

				template class SerialServoMotor<Communicator::SerialController::Dynamixel, MX28>;
			}
		}
	}
}

