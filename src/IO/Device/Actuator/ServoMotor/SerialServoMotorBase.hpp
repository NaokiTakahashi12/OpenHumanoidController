
/**
  *
  * @file SerialServoMotorBase.hpp
  * @brief Serial communication servo motor base class
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include <vector>
#include <unordered_map>

#include "../../Communicator/SerialFlowScheduler.hpp"

namespace IO {
	namespace Device {
		namespace Actuator {
			namespace ServoMotor {
				class SerialServoMotorBase {
					public :
						using SendPacket = Communicator::SerialFlowScheduler::SinglePacket;
						using ID = Communicator::SerialFlowScheduler::Byte;
						using IDList = std::vector<ID>;

						SerialServoMotorBase();

						IDList id_list();
						void id_list_push_back(const ID &);
						void reset_id_list();

					protected :
						std::unique_ptr<IDList> ids;
				};
			}
		}
	}
}


