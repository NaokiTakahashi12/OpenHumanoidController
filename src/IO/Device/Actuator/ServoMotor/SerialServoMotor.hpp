
/**
  *
  * @file SerialServoMotor.hpp
  * @brief Servo motor class
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include "SerialServoMotorBase.hpp"

namespace IO {
	namespace Device {
		namespace Actuator {
			namespace ServoMotor {
				template <class CONTROLLER>
				class SerialServoMotor : public SerialServoMotorBase {
					protected :
						using CommandControllerPtr = std::shared_ptr<CONTROLLER>;

						CommandControllerPtr command_controller;

					public :
						SerialServoMotor(RobotStatus::InformationPtr &);
						SerialServoMotor(const ID &, RobotStatus::InformationPtr &);

						void register_controller(CommandControllerPtr &);

						virtual void ping(),
								     enable_torque(const bool &flag),
							 		 write_gain(const WriteValue &p, const WriteValue &i, const WriteValue &d),
							 		 write_angle(const float &degree);
				};
			}
		}
	}
}

