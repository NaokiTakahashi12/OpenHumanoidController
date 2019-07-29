
/**
  *
  * @file SensorDeviceConfig.cpp
  * @author Naoki Takahashi
  *
  **/

#include "SensorDeviceConfig.hpp"

#include <Tools/ConfigFileOperator/JsonLoader.hpp>

#include <exception>

namespace IO {
	namespace LoadConfig {
		SensorDeviceConfig::SensorDeviceConfig(const std::string &config_filename) {
			this->config_filename = config_filename;
			logger_ptr = std::make_unique<Tools::Log::Logger>();
		}

		SensorDeviceConfig::SensorDeviceConfig(const std::string &config_filename, Tools::Log::LoggerPtr &logger_ptr) {
			this->config_filename = config_filename;
			this->logger_ptr = logger_ptr;
		}

		void SensorDeviceConfig::update() {
			logger_ptr->message(Tools::Log::MessageLevels::trace, "Update sensor device config now");
			if(!imu_config) {
				imu_config = std::make_unique<IMUConfigData>();
			}
			update_imu_config_data();
		}

		void SensorDeviceConfig::force_update() {
			logger_ptr->message(Tools::Log::MessageLevels::trace, "Update sensor device config now");
			imu_config = std::make_unique<IMUConfigData>();
			update_imu_config_data();
		}

		void SensorDeviceConfig::update_imu_config_data() {
			logger_ptr->message(Tools::Log::MessageLevels::trace, "Update config from " + config_filename);
			Tools::ConfigFileOperator::JsonLoader mather_config_file(config_filename);
			std::string sensor_config_filename = mather_config_file.get_parameter<std::string>(imu_config_file_name);

			logger_ptr->message(Tools::Log::MessageLevels::trace, "Load imu config from " + sensor_config_filename);
			set_imu_config_from_jsonfilename(sensor_config_filename);
			print_out_status();
		}

		void SensorDeviceConfig::set_imu_config_from_jsonfilename(const std::string &filename) {
			Tools::ConfigFileOperator::JsonLoader jsonfile(filename);
			imu_config->name = jsonfile.get_parameter<std::string>(use_imu_name);
			imu_config->accel = jsonfile.get_parameter<bool>(imu_enable + accelerometer);
			imu_config->gyro = jsonfile.get_parameter<bool>(imu_enable + gyroscope);
			imu_config->compas = jsonfile.get_parameter<bool>(imu_enable + magnetmeter);
			imu_config->eulerangle = jsonfile.get_parameter<bool>(imu_enable + eulerangle);
			imu_config->quaternion = jsonfile.get_parameter<bool>(imu_enable + quaternion);
			imu_config->heading = jsonfile.get_parameter<bool>(imu_enable + heading);
		}

		void SensorDeviceConfig::print_out_status() {
			logger_ptr->message(Tools::Log::MessageLevels::trace, " IMU name is " + imu_config->name);
			logger_ptr->message(Tools::Log::MessageLevels::trace, " Accelerometers is " + std::to_string(imu_config->accel));
			logger_ptr->message(Tools::Log::MessageLevels::trace, " Gyroscopes is " + std::to_string(imu_config->gyro));
			logger_ptr->message(Tools::Log::MessageLevels::trace, " Magnetometers is " + std::to_string(imu_config->compas));
			logger_ptr->message(Tools::Log::MessageLevels::trace, " EulerAngles is " + std::to_string(imu_config->eulerangle));
			logger_ptr->message(Tools::Log::MessageLevels::trace, " Quaternions is " + std::to_string(imu_config->quaternion));
			logger_ptr->message(Tools::Log::MessageLevels::trace, " Heading is " + std::to_string(imu_config->heading));
		}
	}
}

