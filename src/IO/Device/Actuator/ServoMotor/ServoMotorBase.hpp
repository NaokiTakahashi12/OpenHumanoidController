
/**
  *
  * @file ServoMotorBase.hpp
  * @brief Servo motor base class
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include <vector>

namespace IO {
	namespace Device {
		namespace Actuator {
			namespace ServoMotor {
				class ServoMotorBase {
					public :
						using Angles = std::vector<float>;
						using Speeds = std::vector<float>;
				};
			}
		}
	}
}

