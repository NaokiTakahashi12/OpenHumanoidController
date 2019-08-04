
/**
  *
  * @file Robot.cpp
  * @author Naoki Takahashi
  *
  **/

#include "Robot.hpp"

namespace IO {
	Robot::Robot() {
	}

	Robot::Robot(RobotStatus::InformationPtr &robot_status_information_ptr) {
		robo_info = robot_status_information_ptr;
	}

	Robot::Robot(const int &argc, char **argv) {
		robo_info = std::make_shared<RobotStatus::Information>(argc, argv);
	}

	Robot::~Robot() {
	}

	void Robot::set_config(const std::string &filename) {
		robot_config = std::make_unique<LoadConfig::RobotConfig>(filename, robo_info->logger);
	}

	template <>
	void Robot::update<Robot::UpdateCommands::All>() {
	}

	template <>
	void Robot::update<Robot::UpdateCommands::Motors>() {
	}

	template <>
	void Robot::update<Robot::UpdateCommands::Sensors>() {
	}

	template <>
	void Robot::register_device_map<Robot::RegisterDeviceType::SerialIMU, Robot::IMU>(const RegistMapHash &key, IMU &imu) {
		if(!imu_device_map) {
			imu_device_map = std::make_unique<IMURegistMap>();
		}
		(*imu_device_map)[key] = imu;
	}

	template <>
	void Robot::register_device<Robot::RegisterDeviceType::SerialIMU, Robot::IMU>(IMU &imu_device) {
		this->imu_device = std::make_unique<IMU>(imu_device);
	}
}

