
/**
  *
  * @file Launcher.cpp
  * @author Naoki Takahashi
  *
  **/

#include "Launcher.hpp"

namespace Core {
	Launcher::Launcher(int argc, char **argv) {
		robo_info = std::make_shared<RobotStatus::Information>(argc, argv);
	}

	Launcher::~Launcher() {
	}
}

