
/**
  *
  * @file Information.hpp
  * @brief robot state class
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include <string>
#include <memory>
#include <vector>

#include <Tools/Log/Logger.hpp>

#include "TimeSeries.hpp"

namespace RobotStatus {
	class Information {
		protected :
			using AccelerometersDataType = TimeSeries<Tools::Math::Vector3<float>>;
			using GyroscopesDataType = TimeSeries<Tools::Math::Vector3<float>>;
			using MagnetometersDataType = TimeSeries<Tools::Math::Vector3<float>>;
			using EulerAnglesDataType = TimeSeries<Tools::Math::Vector3<float>>;
			using QuaternionsDataType = TimeSeries<Tools::Math::Vector4<float>>;
			using HeadingDataType = TimeSeries<float>;

			using ServoDataType = std::vector<TimeSeries<float>>;

		public :
			Information();
			Information(const int &argc, char **argv);

			using ConfigName = std::string;

			std::unique_ptr<AccelerometersDataType> accelerometers_data;
			std::unique_ptr<GyroscopesDataType> gyroscopes_data;
			std::unique_ptr<MagnetometersDataType> magnetometers_data;
			std::unique_ptr<EulerAnglesDataType> eulerangles_data;
			std::unique_ptr<QuaternionsDataType> quaternions_data;
			std::unique_ptr<HeadingDataType> heading_data;

			std::unique_ptr<ServoDataType> read_servo_data;
			std::unique_ptr<ServoDataType> write_servo_data;

			std::shared_ptr<Tools::Log::Logger> logger;

			enum class RobotType : char {
				Null,
				Humanoid,
				Cart
			};

			template <RobotType>
			void set_config_filename(const ConfigName &);

			template <RobotType>
			ConfigName get_config_filename() const;

			enum class DeviceType : char {
				Sensor,
				Actuator
			};

			template <DeviceType>
			void set_config_filename(const ConfigName &);

			template <DeviceType>
			ConfigName get_config_filename() const;

			template <DeviceType>
			bool empty_config_filename() const;

			void register_logger(int argc, char **argv);

			void create_imu_data_space(const int &size = 18),
				 create_accel_data_space(const int &size = 18),
				 create_gyro_data_space(const int &size = 18),
				 create_magnet_data_space(const int &size = 18),
				 create_euler_data_space(const int &size = 18),
				 create_quat_data_space(const int &size = 18),
				 create_head_data_space(const int &size = 18);

			void create_servo_data_space(const int &size);

		private :
			RobotType robot_type;

			std::unique_ptr<ConfigName> robot_configname,
										sensor_configname,
										actuator_configname;
	};

	using InformationPtr = std::shared_ptr<Information>;
}

