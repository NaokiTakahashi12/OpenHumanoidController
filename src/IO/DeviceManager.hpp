
/**
  *
  * @file DeviceManager.hpp
  * @brief I/O device manager class
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include <unordered_map>

#include <RobotStatus/Information.hpp>
#include <Tools/Log/Logger.hpp>

#include "LoadConfig/RobotConfig.hpp"

#include "Device/Sensor/IMU/InertialMeasurementUnit.hpp"
#include "Device/ControlBoard/SerialControlBoard.hpp"
#include "Device/Actuator/ServoMotor/SerialServoMotor.hpp"

namespace IO {
	class DeviceManager final {
		public :
			DeviceManager(RobotStatus::InformationPtr &);
			DeviceManager(RobotStatus::InformationPtr &, Tools::Log::LoggerPtr &);

			DeviceManager(const DeviceManager &) = delete;

			~DeviceManager();

			void spawn_device();

			void launch_device();

		private :
			using ID = int;
			using SerialID = ID;

			using SerialControllerPtr = std::shared_ptr<Communicator::SerialController::SerialControllerBase>;
			using SerialControllerMap = std::unordered_map<SerialID, SerialControllerPtr>;

			using IMUPtr = std::unique_ptr<Device::Sensor::IMU::InertialMeasurementUnit>;

			using ControlBoardPtr = std::unique_ptr<Device::ControlBoard::SerialControlBoard>;

			using ServoMotorPtr = std::unique_ptr<Device::Actuator::ServoMotor::SerialServoMotor>;
			using ServoMotorMap = std::unordered_map<ID, ServoMotorPtr>;

			RobotStatus::InformationPtr robo_info;
			Tools::Log::LoggerPtr logger_ptr;

			std::unique_ptr<LoadConfig::RobotConfig> load_robot_config;

			SerialControllerMap serial_control_map;
			IMUPtr imu;
			ControlBoardPtr control_board;
			ServoMotorMap servo_motor_map;

			std::unique_ptr<std::thread> update_servo_angle_thread;

			void update_config();
			void force_update_config();

			void spawn_serial_controller();
			void spawn_sensor_device();
			void spawn_control_board_device();
			void spawn_actuator_device();

			void spawn_imu_device();
			void spawn_servo_motor_device();

			void launch_sensor_device();
			void launch_actuator_with_control_board();

			bool is_exist_serial_id(const SerialID &);
	};
}

