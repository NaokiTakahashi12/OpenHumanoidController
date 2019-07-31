
/**
  *
  * @file InertialMeasurementUnit.cpp
  * @author Naoki Takahashi
  *
  **/

#include "InertialMeasurementUnit.hpp"

#include <stdexcept>

namespace IO {
	namespace Device {
		namespace Sensor {
			namespace IMU {
				void InertialMeasurementUnit::enable(const Streams &) {
					throw std::logic_error("Unoverride from IO::Device::Sensor::IMU::InertialMeasurementUnit");
				}

				void InertialMeasurementUnit::enable_all() {
					throw std::logic_error("Unoverride from IO::Device::Sensor::IMU::InertialMeasurementUnit");
				}

				void InertialMeasurementUnit::launch() {
					throw std::logic_error("Unoverride from IO::Device::Sensor::IMU::InertialMeasurementUnit");
				}

				void InertialMeasurementUnit::async_launch() {
					throw std::logic_error("Unoverride from IO::Device::Sensor::IMU::InertialMeasurementUnit");
				}
			}
		}
	}
}

