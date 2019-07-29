
/**
  *
  * @file DeviceManager.hpp
  * @brief I/O device manager class
  * @author Naoki Takahashi
  *
  **/

#pragma once

#if __has_include(<variant>)
#include <variant>
#endif

#include <unordered_map>

#include <RobotStatus/Information.hpp>
#include <Tools/Log/Logger.hpp>

#include "LoadConfig/RobotConfig.hpp"

namespace IO {
	class DeviceManager final {
		public :
			DeviceManager(RobotStatus::InformationPtr &);
			DeviceManager(RobotStatus::InformationPtr &, Tools::Log::LoggerPtr &);

			DeviceManager(const DeviceManager &) = delete;

			void update_config();
			void force_update_config();

			void device_thread_launch();

		private :
			RobotStatus::InformationPtr robo_info_ptr;
			Tools::Log::LoggerPtr logger_ptr;

			std::unique_ptr<LoadConfig::RobotConfig> load_robot_config;
	};
}

