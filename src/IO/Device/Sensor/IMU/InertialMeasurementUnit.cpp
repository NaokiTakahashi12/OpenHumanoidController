
/**
  *
  * @file InertialMeasurementUnit.cpp
  * @author Naoki Takahashi
  *
  **/

#include "InertialMeasurementUnit.hpp"

#include <stdexcept>

#include "../../../Communicator/SerialController/Simple.hpp"

namespace IO {
	namespace Device {
		namespace Sensor {
			namespace IMU {
				InertialMeasurementUnit::InertialMeasurementUnit(RobotStatus::InformationPtr &robot_status_information_ptr) {
					robo_info = robot_status_information_ptr;
				}

				InertialMeasurementUnit::~InertialMeasurementUnit() {
				}

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

				void InertialMeasurementUnit::port_name(const std::string &new_device_port_name) {
					device_port_name = new_device_port_name;
				}

				std::string InertialMeasurementUnit::port_name() const {
					return device_port_name;
				}
			}
		}
	}
}

