
/**
  *
  * @file SerialServoMotor.hpp
  * @brief Servo motor class
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include "../../SerialDeviceBase.hpp"

#include <RobotStatus/Information.hpp>

#include "../../../Communicator/SerialReturnPacket.hpp"
#include "../../../Communicator/SerialController/SerialControllerBase.hpp"

namespace IO {
	namespace Device {
		namespace Actuator {
			namespace ServoMotor {
				class SerialServoMotor : public SerialDeviceBase<Communicator::SerialController::SerialControllerBase> {
					protected :
						using SendPacket = Communicator::SerialFlowScheduler::SinglePacket;
						using WriteValue = Communicator::SerialFlowScheduler::Byte;

					public :
						using ID = Communicator::SerialReturnPacket::PacketID;

						SerialServoMotor(RobotStatus::InformationPtr &);
						SerialServoMotor(const ID &, RobotStatus::InformationPtr &);

						virtual ~SerialServoMotor();

						virtual void ping(),
								     enable_torque(const bool &flag),
							 		 write_gain(const WriteValue &p, const WriteValue &i, const WriteValue &d),
							 		 write_angle(const float &degree);

						ID &id() const;
						void id(const ID &);

					protected :
						RobotStatus::InformationPtr robo_info;

					private :
						std::unique_ptr<ID> device_id;

				};
			}
		}
	}
}

