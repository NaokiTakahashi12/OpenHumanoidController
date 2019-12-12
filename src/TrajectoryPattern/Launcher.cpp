
/**
  *
  * @file Launcher.cpp
  * @author Naoki Takahashi
  *
  **/

#include "Launcher.hpp"

#include "ConfigManager.hpp"

namespace TrajectoryPattern {
	Launcher::Launcher(RobotStatus::InformationPtr &robot_status_information_ptr) {
		robo_info = robot_status_information_ptr;

		robo_info->create_com_trajectory_data_space(18);
		robo_info->create_footprints_data_space();

		liner_inverted_pendulum = std::make_unique<LinerInvertedPendulum::WalkFragments>(robo_info);
	}

	Launcher::~Launcher() {
		if(async_update_com_trajectory_thread) {
			async_update_com_trajectory_thread->join();
		}
	}

	void Launcher::set_config_file(const std::string &dir, const std::string &filename) {
		if(dir.empty()) {
			throw std::invalid_argument("Failed dir name empty (" + dir + ") from TrajectoryPattern::Launcher");
		}
		if(filename.empty()) {
			throw std::invalid_argument("Failed filename name empty from TrajectoryPattern::Launcher");
		}

		config_dir = dir;
		config_filename = filename;

		initialize();
	}

	bool Launcher::is_computing_now() const {
		return computing_now.load();
	}

	void Launcher::wait_for_computing() const {
		while(is_computing_now()) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			std::this_thread::yield();
		}
	}

	void Launcher::operator ()() {
		if(async_update_com_trajectory_thread) {
			throw std::runtime_error("Already launch async thread from TrajectoryPattern::Launcher");
		}
		async_update_com_trajectory_thread = std::make_unique<std::thread>(
			[this]() {
				static RobotStatus::TimeSeriesData<int>::TimestampType update_time = 0;

				if(update_time < robo_info->left_footprint->latest().timestamp) {
					computing_now.store(true);

					liner_inverted_pendulum->set_com_hight(com_hight);
					liner_inverted_pendulum->set_footprint_list(
						robo_info->left_footprint->latest().value,
						robo_info->right_footprint->latest().value,
						true
					);

					liner_inverted_pendulum->compute(plane_contact_time, minimize_a, minimize_b);

					update_time = robo_info->left_footprint->latest().timestamp;
					computing_now.store(false);
				}
				else {
					std::this_thread::yield();
				}
			}
		);
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

	void Launcher::initialize() {
		const std::string using_algotithm_key = "Walk fragments.";
		ConfigManager config(config_dir, config_filename);

		minimize_a = config.get_value<float>(using_algotithm_key + "Minimize.A");
		minimize_b = config.get_value<float>(using_algotithm_key + "Minimize.B");
		plane_contact_time = config.get_value<float>(using_algotithm_key + "Foot contact time[ms]") * 1e-3;
		com_hight = config.get_value<float>(using_algotithm_key + "CoM hight[mm]") * 1e-3;
	}
}

