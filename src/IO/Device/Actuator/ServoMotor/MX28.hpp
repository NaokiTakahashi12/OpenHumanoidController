
/**
  *
  * @file MX28.hpp
  * @brief MX28 communication class
  * @auther Naoki Takahashi
  *
  **/

#pragma once

#include "SerialServoMotor.hpp"

#include "../../../Communicator/SerialController/Dynamixel.hpp"

namespace IO {
	namespace Device {
		namespace Actuator {
			namespace ServoMotor {
				class MX28 final : public SerialServoMotor {
					public :
						MX28(RobotStatus::InformationPtr &);
						MX28(const ID &, RobotStatus::InformationPtr &);

						~MX28();

						static std::string get_key();

						void ping() override final,
							 enable_torque(const bool &flag) override final,
							 write_gain(const WriteValue &p, const WriteValue &i, const WriteValue &d) override final,
							 write_angle(const float &degree) override final;

					private :
						void write_p_gain(const WriteValue &gain),
							 write_i_gain(const WriteValue &gain),
							 write_d_gain(const WriteValue &gain);

					   	SendPacket create_switch_torque_packet(const bool &flag, const ID &id) const,
								   create_write_p_gain_packet(const WriteValue &gain, const ID &id) const,
								   create_write_i_gain_packet(const WriteValue &gain, const ID &id) const,
								   create_write_d_gain_packet(const WriteValue &gain, const ID &id) const,
								   create_angle_write_packet(const float &degree, const ID &id) const;
				};
			}
		}
	}
}

