
/**
  *
  * @file InertialMeasurementUnit.hpp
  * @brief IMU utility class
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include <RobotStatus/Information.hpp>

#include "InertialMeasurementUnitBase.hpp"

namespace IO {
	namespace Device {
		namespace Sensor {
			namespace IMU {
				class InertialMeasurementUnit : public InertialMeasurementUnitBase {
					public :
						virtual ~InertialMeasurementUnit();

						virtual void enable(const Streams &),
									 enable_all();

						virtual void launch(),
									 async_launch();
				};
			}
		}
	}
}

