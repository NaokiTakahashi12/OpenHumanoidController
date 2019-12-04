
/**
  *
  * @file SerialDeviceSelector.cpp
  * @author Naoki Takahashi
  *
  **/

#include "SerialDeviceSelector.hpp"

#include "Device/Actuator/ServoMotor/SerialServoMotor.hpp"
#include "Device/ControlBoard/SerialControlBoard.hpp"
#include "Device/Sensor/IMU/InertialMeasurementUnit.hpp"

#include "Device/Actuator/ServoMotor/MX28.hpp"
#include "Device/Actuator/ServoMotor/B3MSC1170A.hpp"
#include "Device/ControlBoard/CM730.hpp"
#include "Device/Sensor/IMU/VMU931.hpp"

namespace IO {
	template <typename Base>
	SerialDeviceSelector<Base>::SerialDeviceSelector(RobotStatus::InformationPtr &robot_status_information_ptr) {
		robo_info = robot_status_information_ptr;

		default_register();
	}

	template <typename Base>
	SerialDeviceSelector<Base>::~SerialDeviceSelector() {
	}

	template <>
	void SerialDeviceSelector<Device::Actuator::ServoMotor::SerialServoMotor>::default_register() {
		this->register_object(
			{
				Device::Actuator::ServoMotor::MX28::get_key(),
				std::make_unique<Device::Actuator::ServoMotor::MX28>(robo_info),
			}
		);
		this->register_object(
			{
				Device::Actuator::ServoMotor::B3MSC1170A::get_key(),
				std::make_unique<Device::Actuator::ServoMotor::B3MSC1170A>(robo_info),
			}
		);
	}

	template <>
	void SerialDeviceSelector<Device::ControlBoard::SerialControlBoard>::default_register() {
		this->register_object(
			{
				Device::ControlBoard::CM730::get_key(),
				std::make_unique<Device::ControlBoard::CM730>(robo_info),
			}
		);
	}

	template <>
	void SerialDeviceSelector<Device::Sensor::IMU::InertialMeasurementUnit>::default_register() {
		this->register_object(
			{
				Device::Sensor::IMU::VMU931::get_key(),
				std::make_unique<Device::Sensor::IMU::VMU931>(robo_info)
			}
		);
	}

	template class SerialDeviceSelector<Device::Actuator::ServoMotor::SerialServoMotor>;
	template class SerialDeviceSelector<Device::ControlBoard::SerialControlBoard>;
	template class SerialDeviceSelector<Device::Sensor::IMU::InertialMeasurementUnit>;
}

