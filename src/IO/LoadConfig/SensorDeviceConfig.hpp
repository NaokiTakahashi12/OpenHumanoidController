
/**
  *
  * @file SensorDeviceConfig.hpp
  * @brief Load sensor setting class
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include "LoadConfigBase.hpp"

#include <Tools/Log/Logger.hpp>

#include "SerialPortConfig.hpp"

namespace IO {
	namespace LoadConfig {
		class SensorDeviceConfig : public LoadConfigBase {
			public :
				SensorDeviceConfig(const std::string &config_dir, const std::string &config_filename);
				SensorDeviceConfig(const std::string &config_dir, const std::string &config_filename, Tools::Log::LoggerPtr &);

				SensorDeviceConfig(const SensorDeviceConfig &) = delete;

				struct IMUConfigData {
					// Enable list
					bool accel,
						 gyro,
						 compas,
						 eulerangle,
						 quaternion,
						 heading;

					std::string name;

					SerialPortConfig::SerialControlData::SerialID serial_id;
				};

				std::unique_ptr<IMUConfigData> imu_config;

				void update() override final;
				void force_update() override final;

			private :
				std::string config_dir,
							config_filename;
				Tools::Log::LoggerPtr logger_ptr;

				// Config file value identities
				const std::string imu_config_file_name = "IMU config file",
								  use_imu_name = "IMU",
								  serial_id_key = "Serial ID",
							   	  imu_enable = "Enable list.",
							      accelerometer = "Accelerometers",
							      gyroscope = "Gyroscopes",
							      magnetmeter = "Magnetometers",
							      eulerangle = "EulerAngles",
							      quaternion = "Quaternions",
							      heading = "Heading";

				bool is_registed_configfile_name();

				void update_imu_config_data();
				void set_imu_config_from_jsonfilename(const std::string &);

				void print_out_status();
		};
	}
}

