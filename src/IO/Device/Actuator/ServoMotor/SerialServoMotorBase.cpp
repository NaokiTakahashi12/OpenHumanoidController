
/**
  *
  * @file SerialServoMotorBase.cpp
  * @author Naoki Takahashi
  *
  **/

#include "SerialServoMotorBase.hpp"

namespace IO {
	namespace Device {
		namespace Actuator {
			namespace ServoMotor {
				SerialServoMotorBase::SerialServoMotorBase() {
					ids = std::make_unique<IDList>();
				}

				SerialServoMotorBase::IDList SerialServoMotorBase::id_list() {
					return *ids;
				}

				void SerialServoMotorBase::id_list_push_back(const ID &id) {
					ids->push_back(id);
				}

				void SerialServoMotorBase::reset_id_list() {
					if(ids) {
						ids.reset();
						ids = std::make_unique<IDList>();
					}
				}
			}
		}
	}
}

