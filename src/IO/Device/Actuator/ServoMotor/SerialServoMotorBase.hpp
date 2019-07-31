
/**
  *
  * @file SerialServoMotorBase.hpp
  * @brief Serial communication servo motor base class
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include <RobotStatus/Information.hpp>

#include "../../Communicator/SerialReturnPacket.hpp"

namespace IO {
	namespace Device {
		namespace Actuator {
			namespace ServoMotor {
				class SerialServoMotorBase {
					public :
						using SendPacket = Communicator::SerialFlowScheduler::SinglePacket;
						using WriteValue = Communicator::SerialFlowScheduler::Byte;
						using ID = Communicator::SerialReturnPacket::PacketID;

						SerialServoMotorBase(RobotStatus::InformationPtr &);
						SerialServoMotorBase(const ID &, RobotStatus::InformationPtr &);

						SerialServoMotorBase(const SerialServoMotorBase &);

						virtual ~SerialServoMotorBase();

						ID &id() const;
						void id(const ID &) const;

					protected :
						RobotStatus::InformationPtr robo_info;

					private :
						std::unique_ptr<ID> device_id;
				};
			}
		}
	}
}


