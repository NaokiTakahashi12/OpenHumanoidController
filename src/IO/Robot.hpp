
/**
  *
  * @file Robot.hpp
  * @brief I/O Robot interface class
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include <RobotStatus/Information.hpp>
#include <Tools/NonCopyable.hpp>

#include "LoadConfig/RobotConfig.hpp"
#include "Device/Actuator/ServoMotor/SerialServoMotor.hpp"
#include "Device/ControlBoard/SerialControlBoard.hpp"
#include "Communicator/SerialController/Dynamixel.hpp"

namespace IO {
	class Robot : Tools::NonCopyable {
		public :
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

		protected :
			RobotStatus::InformationPtr robo_info;
			
			std::unique_ptr<LoadConfig::RobotConfig> robot_config;

		private :
			void thread_launcher();

			void load_config();
	};
}

