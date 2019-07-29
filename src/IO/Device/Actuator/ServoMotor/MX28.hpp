
/**
  *
  * @file MX28.hpp
  * @brief MX28 communication class
  * @auther Naoki Takahashi
  *
  **/

#pragma once

#include "SerialServoMotorBase.hpp"

#include "../../../Communicator/SerialController/Dynamixel.hpp"

namespace IO {
	namespace Device {
		namespace Actuator {
			namespace ServoMotor {
				class MX28 final : public SerialServoMotorBase {
					public :
						using SerialController = Communicator::SerialController::Dynamixel;

						MX28();

						void register_controller(std::shared_ptr<SerialController> &);

						void enable_torque(const bool &flag, const ID &id),
							 write_p_gain(const unsigned short &gain, const ID &id),
							 write_angle(const float &degree, const ID &id),
							 ping(const ID &id);

					private :
					   SendPacket create_switch_torque_packet(const bool &flag, const ID &id),
								  create_write_p_gain_packet(const unsigned short &gain, const ID &id),
								  create_angle_write_packet(const float &degree, const ID &id);

					   std::shared_ptr<SerialController> serial_controller;
				};
			}
		}
	}
}

