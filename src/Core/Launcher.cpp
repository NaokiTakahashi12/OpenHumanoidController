
/**
  *
  * @file Launcher.cpp
  * @author Naoki Takahashi
  *
  **/

#include "Launcher.hpp"

#include <iostream>

namespace Core {
	Launcher::Launcher(int argc, char **argv) {
		robo_info = std::make_shared<RobotStatus::Information>(argc, argv);
		robo_info->logger->start_loger_thread();

		begin_point << 1e-6, 1e-6, 1e-6;
		goal_point << 1, 1e-6, 1e-6;

		robo_info->create_footprints_data_space();

		launch_up_humanoid_footprint_manager();
		launch_up_kinematics();
		launch_up_io();
		launch_up_trajectory_pattern_generator();
	}

	Launcher::~Launcher() {
		if(command_thread_for_kinematics) {
			command_thread_for_kinematics->join();
		}
	}

	void Launcher::launch_up_humanoid_footprint_manager() {
		robo_info->logger->message(Tools::Log::MessageLevels::info, "Launch up footprint thread");

		footstep_planner = FootStepPlanner::HumanoidFootprintManager<float>::make_ptr();
		footstep_planner = std::make_unique<FootStepPlanner::HumanoidFootprintManager<float>>();

		footstep_planner->choice_footprint_planner("../../FootStepPlanner/build/footstep_planner.conf.json");

		footstep_planner->set_begin(begin_point.x(), begin_point.y(), begin_point.z(), 0, 0, 0);
		footstep_planner->set_goal(goal_point.x(), goal_point.y(), goal_point.z(), 0, 0, 0);

		footstep_planner->make_full_footprint();

		Tools::Math::MatrixX<float> left_footprint, right_footprint;
		{
			const auto footprint_list = footstep_planner->get_footprint_list();

			int footprint_size = footprint_list.size();
			if(!(footprint_size % 2)) {
				footprint_size = footprint_size * 0.5 - 1;
			}
			else {
				footprint_size = footprint_size * 0.5;
			}

			left_footprint = left_footprint.Zero(3, footprint_size);
			right_footprint = right_footprint.Zero(3, footprint_size);

			Tools::Math::Vector3<float> left_cache, right_cache;

			int left_i = 0, right_i = 0;
			for(const auto &fp : footprint_list) {
				if(left_cache != fp.left && left_i < footprint_size) {
					left_footprint.block<3, 1>(0, left_i) = fp.left;
					left_cache = fp.left;
					left_i ++;
				}
				if(right_cache != fp.right && right_i < footprint_size) {
					right_footprint.block<3, 1>(0, right_i) = fp.right;
					right_cache = fp.right;
					right_i ++;
				}
			}
		}
		
		robo_info->left_footprint->set(left_footprint);
		robo_info->right_footprint->set(right_footprint);
	}

	void Launcher::launch_up_kinematics() {
		robo_info->logger->message(Tools::Log::MessageLevels::info, "Launch up Kinematics thread");

		kinematics_launcher = std::make_unique<Kinematics::Launcher<double>>(robo_info, "../../Kinematics/build/", "kinematics.conf.json");
		kinematics_launcher->initialize();

		(*kinematics_launcher)();

		std::this_thread::sleep_for(std::chrono::milliseconds(20));

		auto control_point_map = kinematics_launcher->get_control_point_map();
		auto parameters = kinematics_launcher->get_parameters();
		const auto left_foot_id = 20;
		const auto right_foot_id = 14;
		Kinematics::Quantity::SpatialPoint<double> initial_left_foot_point, initial_right_foot_point;
		Kinematics::Quantity::SpatialPoint<double> ready_left_foot_point, ready_right_foot_point;

		{
			const auto lock = std::lock_guard<std::mutex>(kinematics_launcher->get_mutex());
			initial_left_foot_point = control_point_map->get_point(left_foot_id);
			initial_right_foot_point = control_point_map->get_point(right_foot_id);
		}

		{
			const auto down_com = 0.04;
			const auto max_count = 5;
			for(auto i = 0; i < max_count; i ++) {
				const auto minimum_diff = down_com / max_count;
				{
					const auto lock = std::lock_guard<std::mutex>(kinematics_launcher->get_mutex());
					control_point_map->add(left_foot_id, Kinematics::Quantity::SpatialPoint<double>().point(0, 0, minimum_diff));
					control_point_map->add(right_foot_id, Kinematics::Quantity::SpatialPoint<double>().point(0, 0, minimum_diff));
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(250));
			}
			{
				const auto lock = std::lock_guard<std::mutex>(kinematics_launcher->get_mutex());
				ready_left_foot_point = control_point_map->get_point(left_foot_id);
				ready_right_foot_point = control_point_map->get_point(right_foot_id);
			}
		}
		auto logger = robo_info->logger;

		closeable.store(false);

		command_thread_for_kinematics = std::make_unique<std::thread>(
			[=]() {
				static Tools::Math::Vector3<double> left_trajectory, right_trajectory;

				while(!closeable.load()) {
					if(robo_info->left_foot_trajectory && robo_info->right_foot_trajectory) {
						{
							const auto lt = robo_info->left_foot_trajectory->latest().value.cast<double>();
							const auto rt = robo_info->right_foot_trajectory->latest().value.cast<double>();

							if(left_trajectory == lt || right_trajectory == rt) {
								std::this_thread::yield();
								continue;
							}

							left_trajectory = lt;
							right_trajectory = rt;
						}

						Kinematics::Quantity::SpatialPoint<double> command_left_point, command_right_point;

						std::stringstream ss;
						ss << left_trajectory.transpose() << " : ";
						ss << right_trajectory.transpose();
						robo_info->logger->message(Tools::Log::MessageLevels::debug, ss.str());

						command_left_point.point(
							left_trajectory
							+ ready_left_foot_point.point()
						);
						command_right_point.point(
							right_trajectory
							+ ready_right_foot_point.point()
						);

						{
							const auto lock = std::lock_guard<std::mutex>(kinematics_launcher->get_mutex());
							control_point_map->update(left_foot_id, command_left_point);
							control_point_map->update(right_foot_id, command_right_point);
						}
					}
					else {
						std::this_thread::yield();
					}
					std::this_thread::sleep_for(std::chrono::microseconds(100));
				}

				left_trajectory = robo_info->left_foot_trajectory->latest().value.cast<double>();
				right_trajectory = robo_info->right_foot_trajectory->latest().value.cast<double>();

				Kinematics::Quantity::SpatialPoint<double> command_left_point, command_right_point;
				command_left_point.point(left_trajectory + ready_left_foot_point.point());
				command_right_point.point(right_trajectory + ready_right_foot_point.point());
				{
					const auto lock = std::lock_guard<std::mutex>(kinematics_launcher->get_mutex());
					control_point_map->update(left_foot_id, command_left_point);
					control_point_map->update(right_foot_id, command_right_point);
				}
			}
		);
	}

	void Launcher::launch_up_io() {
		robo_info->logger->message(Tools::Log::MessageLevels::info, "Launch up IO thread");

		robo_info->set_config_filename<RobotStatus::Information::RobotType::Humanoid>("robot.conf.json");

		io_device_manager = std::make_unique<IO::DeviceManager>("../../IO/build/", "robot.conf.json", robo_info, robo_info->logger);

		/*
		io_device_manager->spawn_device();
		io_device_manager->launch_device();
		*/

		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}

	void Launcher::launch_up_trajectory_pattern_generator() {
		robo_info->logger->message(Tools::Log::MessageLevels::info, "Launch up TrajectoryPattern thread");

		trajectory_pattern_generator = std::make_unique<TrajectoryPattern::Launcher>(robo_info);
		trajectory_pattern_generator->set_config_file("../../TrajectoryPattern/build/", "trajectory_pattern.json");

		(*trajectory_pattern_generator)();
		trajectory_pattern_generator->wait_for_computing();

		std::this_thread::sleep_for(std::chrono::milliseconds(10));

		closeable.store(true);

		/*
		{
			std::vector<double> x, y;
			for(auto &&c : trajectory_pattern_generator->liner_inverted_pendulum->com_line) {
				x.push_back(c.x());
				y.push_back(c.y());
			}
		}
		*/
	}
}

