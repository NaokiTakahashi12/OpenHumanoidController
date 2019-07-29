
/**
  *
  * @file InertialMeasurementUnit.cpp
  * @author Naoki Takahashi
  *
  **/

#include "InertialMeasurementUnit.hpp"

namespace IO {
	namespace Device {
		namespace Sensor {
			namespace IMU {
				template <class IMUTYPE>
				InertialMeasurementUnit<IMUTYPE>::InertialMeasurementUnit(RobotStatus::InformationPtr &robot_status_information_ptr) {
					imu_device = std::make_unique<IMUTYPE>(robot_status_information_ptr);
				}

				template <class IMUTYPE>
				void InertialMeasurementUnit<IMUTYPE>::enable(const typename IMUTYPE::Streams &stream) {
					imu_device->enable(stream);
				}

				template <class IMUTYPE>
				void InertialMeasurementUnit<IMUTYPE>::enable_all() {
					imu_device->enable_all();
				}

				template <class IMUTYPE>
				void InertialMeasurementUnit<IMUTYPE>::port_name(const std::string &name) {
					imu_device->port_name(name);
				}

				template <class IMUTYPE>
				void InertialMeasurementUnit<IMUTYPE>::launch() {
					imu_device->launch();
				}

				template <class IMUTYPE>
				void InertialMeasurementUnit<IMUTYPE>::async_launch() {
					imu_device->async_launch();
				}

				template <class IMUTYPE>
				typename InertialMeasurementUnit<IMUTYPE>::IMUDevicePtr InertialMeasurementUnit<IMUTYPE>::get_device() {
					return std::move(imu_device);
				}

				template class InertialMeasurementUnit<VMU931>;
			}
		}
	}
}

