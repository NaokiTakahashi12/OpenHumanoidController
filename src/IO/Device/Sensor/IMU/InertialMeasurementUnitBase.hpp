
/**
  *
  * @file InertialMeasurementUnitBase.hpp
  * @brief IMU device base class
  * @author Naoki Takahashi
  *
  **/

#pragma once

namespace IO {
	namespace Device {
		namespace Sensor {
			namespace IMU {
				class InertialMeasurementUnitBase {
					public:
						enum class Streams : char {
							Accelerometers = 'a',
							Gyroscopes = 'g',
							Magnetometers = 'c',
							EulerAngles = 'e',
							Quaternions = 'q',
							Heading = 'h'
						};
				};
			}
		}
	}
}
