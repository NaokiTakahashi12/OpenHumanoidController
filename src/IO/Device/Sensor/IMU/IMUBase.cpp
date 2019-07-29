
/**
  *
  * @file IMUBase.cpp
  * @author Naoki Takahashi
  *
  **/

#include "IMUBase.hpp"

#include <stdexcept>

namespace IO {
	namespace Device {
		namespace Sensor {
			namespace IMU {
				void IMUBase::enable(const Streams &) {
					throw std::runtime_error("Unoverride");
				}

				void IMUBase::enable_all() {
					throw std::runtime_error("Unoverride");
				}
			}
		}
	}
}
