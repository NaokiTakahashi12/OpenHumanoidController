
#include <exception>
#include <iostream>

#include <RobotStatus/Information.hpp>

#include <Tools/Log/Logger.hpp>

#include "LinerInvertedPendulum/WalkFragments.hpp"

#include "Launcher.hpp"

int  main(int argc, char **argv) {
	auto robo_info = std::make_shared<RobotStatus::Information>(argc, argv);

	auto logger = robo_info->logger;
	logger->start_loger_thread();

	try {
		std::vector<Eigen::Vector2f> footprint;

		TrajectoryPattern::Launcher launcher(robo_info);
		launcher.set_config_file("./", "trajectory_pattern.json");

		footprint.push_back(TrajectoryPattern::LinerInvertedPendulum::WalkFragments::Vector2(-0.484357, 0.0887688));
		footprint.push_back(TrajectoryPattern::LinerInvertedPendulum::WalkFragments::Vector2(-0.471544, -0.010407));
		footprint.push_back(TrajectoryPattern::LinerInvertedPendulum::WalkFragments::Vector2(-0.453044, 0.0878668));
		footprint.push_back(TrajectoryPattern::LinerInvertedPendulum::WalkFragments::Vector2(-0.4383, -0.0110404));
		footprint.push_back(TrajectoryPattern::LinerInvertedPendulum::WalkFragments::Vector2(-0.413711, 0.0858893));
		footprint.push_back(TrajectoryPattern::LinerInvertedPendulum::WalkFragments::Vector2(-0.389877, -0.0112289));
		footprint.push_back(TrajectoryPattern::LinerInvertedPendulum::WalkFragments::Vector2(-0.353222, 0.0818111));
		footprint.push_back(TrajectoryPattern::LinerInvertedPendulum::WalkFragments::Vector2(-0.314301, -0.0103039));
		footprint.push_back(TrajectoryPattern::LinerInvertedPendulum::WalkFragments::Vector2(-0.259181, 0.0731334));
		footprint.push_back(TrajectoryPattern::LinerInvertedPendulum::WalkFragments::Vector2(-0.198642, -0.00645927));
		footprint.push_back(TrajectoryPattern::LinerInvertedPendulum::WalkFragments::Vector2(-0.1208, 0.0563144));
		footprint.push_back(TrajectoryPattern::LinerInvertedPendulum::WalkFragments::Vector2(-0.0442811, -0.00806664));
		footprint.push_back(TrajectoryPattern::LinerInvertedPendulum::WalkFragments::Vector2(0.0366206, 0.0507119));
		footprint.push_back(TrajectoryPattern::LinerInvertedPendulum::WalkFragments::Vector2(0.112624, -0.0142763));
		footprint.push_back(TrajectoryPattern::LinerInvertedPendulum::WalkFragments::Vector2(0.193526, 0.0445022));
		footprint.push_back(TrajectoryPattern::LinerInvertedPendulum::WalkFragments::Vector2(0.270077, -0.0198399));
		footprint.push_back(TrajectoryPattern::LinerInvertedPendulum::WalkFragments::Vector2(0.350979, 0.0389386));
		footprint.push_back(TrajectoryPattern::LinerInvertedPendulum::WalkFragments::Vector2(0.428516, -0.0242119));
		footprint.push_back(TrajectoryPattern::LinerInvertedPendulum::WalkFragments::Vector2(0.509418, 0.0345666));
		footprint.push_back(TrajectoryPattern::LinerInvertedPendulum::WalkFragments::Vector2(0.588091, -0.027163));
		footprint.push_back(TrajectoryPattern::LinerInvertedPendulum::WalkFragments::Vector2(0.668992, 0.0316156));
		footprint.push_back(TrajectoryPattern::LinerInvertedPendulum::WalkFragments::Vector2(0.748678, -0.028801));
		footprint.push_back(TrajectoryPattern::LinerInvertedPendulum::WalkFragments::Vector2(0.827458, 0.0327931));
		footprint.push_back(TrajectoryPattern::LinerInvertedPendulum::WalkFragments::Vector2(0.888855, -0.0461392));
		footprint.push_back(TrajectoryPattern::LinerInvertedPendulum::WalkFragments::Vector2(0.927704, 0.0460061));
		footprint.push_back(TrajectoryPattern::LinerInvertedPendulum::WalkFragments::Vector2(0.957329, -0.0495051));
		footprint.push_back(TrajectoryPattern::LinerInvertedPendulum::WalkFragments::Vector2(0.972725, 0.0493027));
		footprint.push_back(TrajectoryPattern::LinerInvertedPendulum::WalkFragments::Vector2(0.988675, -0.0494171));
		footprint.push_back(TrajectoryPattern::LinerInvertedPendulum::WalkFragments::Vector2(1.00401, 0.0493995));
		footprint.push_back(TrajectoryPattern::LinerInvertedPendulum::WalkFragments::Vector2(1.0199, -0.0493311));

		{
			const auto footprint_size = footprint.size();
			const auto dimention_size = 3;
			Eigen::MatrixXf left_footprint;
			Eigen::MatrixXf right_footprint;
			left_footprint = Eigen::MatrixXf::Zero(dimention_size, footprint_size / 2);
			right_footprint = Eigen::MatrixXf::Zero(dimention_size, footprint_size / 2);

			unsigned int right_iter = 0;
			unsigned int left_iter = 0;

			for(unsigned int i = 0; i < footprint_size; i ++) {
				if(i % 2) {
					right_footprint.block<dimention_size, 1>(0, right_iter)(0) = footprint.at(i).x();
					right_footprint.block<dimention_size, 1>(0, right_iter)(1) = footprint.at(i).y();

					right_iter ++;
				}
				else {
					left_footprint.block<dimention_size, 1>(0, left_iter)(0) = footprint.at(i).x();
					left_footprint.block<dimention_size, 1>(0, left_iter)(1) = footprint.at(i).y();

					left_iter ++;
				}
			}

			robo_info->left_footprint->set(left_footprint);
			robo_info->right_footprint->set(right_footprint);
		}

		logger->message(Tools::Log::MessageLevels::info, "Stert com line compute");
		launcher();
		launcher.wait_for_computing();
		logger->message(Tools::Log::MessageLevels::info, "Finish com line compute");

		{
			std::vector<double> x, y;
			for(auto &v : footprint) {
				x.push_back(v.x());
				y.push_back(v.y());
			}
		}
		{
			std::vector<double> x, y;
			const auto left_footprint = robo_info->left_footprint->latest().value;
			for(auto i = 0; i < left_footprint.cols(); i ++) {
				x.push_back(left_footprint(0, i));
				y.push_back(left_footprint(1, i));
			}
		}
		{
			std::vector<double> x, y;
			const auto right_footprint = robo_info->right_footprint->latest().value;
			for(auto i = 0; i < right_footprint.cols(); i ++) {
				x.push_back(right_footprint(0, i));
				y.push_back(right_footprint(1, i));
			}
		}
		{
			std::vector<double> x, y;
			const auto modified_left_footprint = robo_info->left_modified_footprint->latest().value;
			for(unsigned int i = 0; i < modified_left_footprint.cols(); i ++) {
				x.push_back(modified_left_footprint(0, i));
				y.push_back(modified_left_footprint(1, i));

				std::stringstream ss;
				ss << modified_left_footprint.block<3, 1>(0, i).transpose();
				logger->message(Tools::Log::MessageLevels::debug, ss.str());
			}
		}
		{
			std::vector<double> x, y;
			const auto modified_right_footprint = robo_info->right_modified_footprint->latest().value;
			for(unsigned int i = 0; i < modified_right_footprint.cols(); i ++) {
				x.push_back(modified_right_footprint(0, i));
				y.push_back(modified_right_footprint(1, i));

				std::stringstream ss;
				ss << modified_right_footprint.block<3, 1>(0, i).transpose();
				logger->message(Tools::Log::MessageLevels::debug, ss.str());
			}
		}
		{
			std::vector<double> x, y;
			const auto com_line = robo_info->com_trajectory->get_all();
			logger->message(Tools::Log::MessageLevels::debug, "CoM recode size is " + std::to_string(com_line.size()));
			for(const auto &v : com_line) {
				x.push_back(v.value.x());
				y.push_back(v.value.y());
			}
		}
	}
	catch(const std::exception &error) {
		std::cerr << error.what() << std::endl;
	}
	return 0;
};

