
/**
  *
  * @file InertialMeasurementUnit.hpp
  * @brief IMU utility class
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include "../../SerialDeviceBase.hpp"

#include <RobotStatus/Information.hpp>

#include "../../../Communicator/SerialController/SerialControllerBase.hpp"

namespace IO {
	namespace Device {
		namespace Sensor {
			namespace IMU {
				class InertialMeasurementUnit : public SerialDeviceBase<Communicator::SerialController::SerialControllerBase> {
					public :
						enum class Streams : char {
							Accelerometers = 'a',
							Gyroscopes = 'g',
							Magnetometers = 'c',
							EulerAngles = 'e',
							Quaternions = 'q',
							Heading = 'h'
						};

						InertialMeasurementUnit(RobotStatus::InformationPtr &);

						virtual ~InertialMeasurementUnit();

						virtual void enable(const Streams &),
									 enable_all();

						virtual void launch(),
									 async_launch();

						void port_name(const std::string &);
						std::string port_name() const;

					protected :
						RobotStatus::InformationPtr robo_info;

					private :
						std::string device_port_name;
				};
			}
		}
	}
}

