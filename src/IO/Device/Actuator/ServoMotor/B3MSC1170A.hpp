
/**
  *
  * @file B3MSC1170A.hpp
  * @author Napki Takahashi
  *
  **/

#pragma once

#include "SerialServoMotor.hpp"

namespace IO {
	namespace Device {
		namespace Actuator {
			namespace ServoMotor {
				class B3MSC1170A final : public SerialServoMotor {
					public :
						B3MSC1170A(RobotStatus::InformationPtr &);
						B3MSC1170A(const ID &, RobotStatus::InformationPtr &);

						~B3MSC1170A();

						static std::string get_key();

						void ping() override final,
							 enable_torque(const bool &flag) override final,
							 write_gain(const WriteValue &p, const WriteValue &i, const WriteValue &d) override final,
							 write_angle(const float &degree) override final;

					private :
						void command_controller_access_assertion();

						void write_gain_packet(const ID &, const WriteValue &p, const WriteValue &i, const WriteValue &d);

						SendPacket create_ping_packet(const ID &),
								   create_enable_torque_packet(const ID &, const bool &),
								   create_write_angle_packet(const ID &, const float &degree);

						SendPacket create_write_p_gain_packet(const ID &, const WriteValue &),
								   create_write_i_gain_packet(const ID &, const WriteValue &),
								   create_write_d_gain_packet(const ID &, const WriteValue &);
				};
			}
		}
	}
}

