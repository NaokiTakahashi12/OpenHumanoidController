
/**
  *
  * @file SensorDeviceConfig.cpp
  * @author Naoki Takahashi
  *
  **/

#include "SensorDeviceConfig.hpp"

#include <Tools/ConfigFileOperator/JsonLoader.hpp>

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

				update_imu_config_data();
			}
		}

		void SensorDeviceConfig::force_update() {
			logger_ptr->message(Tools::Log::MessageLevels::trace, "Update sensor device config now");

			if(imu_config) {
				imu_config.reset();
			}
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
			imu_config->serial_id = jsonfile.get_parameter<int>(serial_id_key);
			imu_config->accel = jsonfile.get_parameter<bool>(imu_enable + accelerometer);
			imu_config->gyro = jsonfile.get_parameter<bool>(imu_enable + gyroscope);
			imu_config->compas = jsonfile.get_parameter<bool>(imu_enable + magnetmeter);
			imu_config->eulerangle = jsonfile.get_parameter<bool>(imu_enable + eulerangle);
			imu_config->quaternion = jsonfile.get_parameter<bool>(imu_enable + quaternion);
			imu_config->heading = jsonfile.get_parameter<bool>(imu_enable + heading);
		}

		void SensorDeviceConfig::print_out_status() {
			std::stringstream ss;

			ss << "Config_IMU:";
			ss << " Name: " << imu_config->name;
			ss << " SerialID: " << imu_config->serial_id;
			ss << " Accelerometers: " << imu_config->accel;
			ss << " Gyroscopes: " << imu_config->gyro;
			ss << " Magnetometers: " << imu_config->compas;
			ss << " EulerAngles: " << imu_config->eulerangle;
			ss << " Quaternions: " << imu_config->quaternion;
			ss << " Heading: " << imu_config->heading;

			logger_ptr->message(Tools::Log::MessageLevels::trace, ss.str());
		}
	}
}

