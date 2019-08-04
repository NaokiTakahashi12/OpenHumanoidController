
/**
  *
  * @file Robot.hpp
  * @brief I/O Robot interface class
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include <unordered_map>

#include <RobotStatus/Information.hpp>
#include <Tools/NonCopyable.hpp>

#include "LoadConfig/RobotConfig.hpp"
#include "Device/Actuator/ServoMotor/SerialServoMotor.hpp"
#include "Device/ControlBoard/SerialControlBoard.hpp"
#include "Device/Sensor/IMU/InertialMeasurementUnit.hpp"
#include "Communicator/SerialController/Dynamixel.hpp"

namespace IO {
	class Robot : Tools::NonCopyable {
		public :
			using RegistMapHash = std::string;

			Robot();
			Robot(RobotStatus::InformationPtr &);
			Robot(const int &argc, char **argv);
			virtual ~Robot();

			void set_config(const std::string &filename);

			enum class UpdateCommands : char {
				All,
				Motors,
				Sensors
			};

			template <UpdateCommands COMMAND>
			void update();

			enum class RegisterDeviceType : char {
				SerialServo,
				SerialIMU
			};

			template <RegisterDeviceType DEVICE, class BASE>
			void register_device_map(const RegistMapHash &, BASE &);

			template <RegisterDeviceType DEVICE, class BASE>
			void register_device(BASE &);

		private :
			using IMU = Device::Sensor::IMU::InertialMeasurementUnit;
			using IMURegistMap = std::unordered_map<RegistMapHash, IMU>;

			RobotStatus::InformationPtr robo_info;

			std::unique_ptr<IMURegistMap> imu_device_map;
			std::unique_ptr<IMU> imu_device;
			
			std::unique_ptr<LoadConfig::RobotConfig> robot_config;

			void thread_launcher();

			void load_config();
	};
}

