
/**
  *
  * @file InertialMeasurementUnitBase.cpp
  * @author Naoki Takahashi
  *
  **/

#include "InertialMeasurementUnitBase.hpp"

namespace IO {
	namespace Device {
		namespace Sensor {
			namespace IMU {
				void InertialMeasurementUnitBase::port_name(const std::string &device_port_name) {
					this->device_port_name = device_port_name;
				}

				std::string InertialMeasurementUnitBase::port_name() const {
					return device_port_name;
				}
			}
		}
	}
}
