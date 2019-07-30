
/**
  *
  * @file B3M.hpp
  * @brief B3M communication class
  * @auther Yasuo Hayashibara
  *
  **/

#pragma once

#include "SerialServoMotorBase.hpp"

#include "../../../Communicator/SerialController/Kondo.hpp"

namespace IO {
	namespace Device {
		namespace Actuator {
			namespace ServoMotor {
				class B3M final : public SerialServoMotorBase {
					public :
						using SerialController = Communicator::SerialController::Kondo;

						B3M();

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
