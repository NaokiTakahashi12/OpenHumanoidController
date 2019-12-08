
/**
  *
  * @file Launcher.hpp
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include <RobotStatus/Information.hpp>

namespace Core {
	class Launcher {
		public :
			Launcher(int argc, char **argv);
			~Launcher();

		private :
			RobotStatus::InformationPtr robo_info;

	};
}

