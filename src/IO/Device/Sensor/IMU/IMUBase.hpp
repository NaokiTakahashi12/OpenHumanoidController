
/**
  *
  * @file IMUBase.hpp
  * @brief IMU device base class
  * @author Naoki Takahashi
  *
  **/

#pragma once

namespace IO {
	namespace Device {
		namespace Sensor {
			namespace IMU {
				class IMUBase {
					public:
						enum class Streams : char {
							Accelerometers = 'a',
							Gyroscopes = 'g',
							Magnetometers = 'c',
							EulerAngles = 'e',
							Quaternions = 'q',
							Heading = 'h'
						};

						virtual void enable(const Streams &);
						virtual void enable_all();
				};
			}
		}
	}
}

