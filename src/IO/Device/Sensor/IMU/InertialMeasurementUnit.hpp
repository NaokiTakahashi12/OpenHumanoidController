
/**
  *
  * @file InertialMeasurementUnit.hpp
  * @brief IMU utility class
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include <RobotStatus/Information.hpp>

#include "IMUBase.hpp"
#include "VMU931.hpp"

namespace IO {
	namespace Device {
		namespace Sensor {
			namespace IMU {
				template <class IMUTYPE>
				class InertialMeasurementUnit final {
					private :
						using IMUDevicePtr = std::unique_ptr<IMUTYPE>;
						IMUDevicePtr imu_device;

					public :
						InertialMeasurementUnit(RobotStatus::InformationPtr &);

						void enable(const typename IMUTYPE::Streams &),
							 enable_all(),
							 port_name(const std::string &name);

						void launch(),
							 async_launch();

						IMUDevicePtr get_device();
				};
			}
		}
	}
}

