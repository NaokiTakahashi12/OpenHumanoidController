
/**
  *
  * @file DeviceManager.cpp
  * @author Naoki Takahashi
  *
  **/

#include "DeviceManager.hpp"

#include <exception>

#include <Tools/ConfigFileOperator/JsonLoader.hpp>

namespace IO {
	DeviceManager::DeviceManager(RobotStatus::InformationPtr &robo_info_ptr) {
		this->robo_info_ptr = robo_info_ptr;
		logger_ptr = std::make_unique<Tools::Log::Logger>();

		load_robot_config = std::make_unique<LoadConfig::RobotConfig>(this->robo_info_ptr->get_config_filename<RobotStatus::Information::RobotType::Humanoid>());
	}

	DeviceManager::DeviceManager(RobotStatus::InformationPtr &robo_info_ptr, Tools::Log::LoggerPtr &logger_ptr) {
		this->robo_info_ptr = robo_info_ptr;
		this->logger_ptr = logger_ptr;

		load_robot_config = std::make_unique<LoadConfig::RobotConfig>(this->robo_info_ptr->get_config_filename<RobotStatus::Information::RobotType::Humanoid>(), this->logger_ptr);
	}

	void DeviceManager::update_config() {
		load_robot_config->update();
	}

	void DeviceManager::force_update_config() {
		load_robot_config->force_update();
	}

	void DeviceManager::device_thread_launch() {
	}
}
