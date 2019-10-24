
#include <cmath>
#include <exception>
#include <iostream>
#include <sstream>
#include <chrono>

#include <Tools/Log/Logger.hpp>
#include <RobotStatus/Information.hpp>

#include "FootprintPlanner/ConstantRangeBasedHumanoid.hpp"

#include "HumanoidFootprintManager.hpp"

int main(int argc, char **argv) {
	auto logger = std::make_shared<Tools::Log::Logger>(argc, argv);
	logger->start_loger_thread();
	try {
		FootStepPlanner::FootprintPlanner::ConstantRangeBasedHumanoid<double>::Ptr planner = FootStepPlanner::FootprintPlanner::ConstantRangeBasedHumanoid<double>::make_ptr();
		FootStepPlanner::HumanoidFootprintManager<double>::Ptr footprint_manager = FootStepPlanner::HumanoidFootprintManager<double>::make_ptr();

		planner->footstep_range(0.1);
		planner->maximum_footprint(100);
		planner->upper_limit_of_forward(0.9);
		planner->lower_limit_of_forward(0.4);
		planner->set_begin(-0.2, 0.2, 0, 0, 0, 0);
		planner->set_goal(1, -0.4, 0, 0, 0, 0);
		planner->register_damping(
			[](const double &x) {
				return std::abs(std::sin(M_PI * x));
			}
		);
		planner->begin_footstep_interval(5);

		footprint_manager->register_footprint_planner(std::move(planner));
		auto start_p = std::chrono::high_resolution_clock::now();
		footprint_manager->generate_footprint();
		auto end_p = std::chrono::high_resolution_clock::now();
		logger->message(Tools::Log::MessageLevels::debug, std::to_string(std::chrono::duration<double, std::milli>(end_p - start_p).count()) + " [ms]");

		{
			std::vector<double> x, y;
			std::vector<std::vector<double>> shadow_x, shadow_y;

			for(auto &&fp : footprint_manager->get_footprint_list()) {
				x.push_back(fp.left.x()), x.push_back(fp.right.x());
				y.push_back(fp.left.y()), y.push_back(fp.right.y());
			}
			{
				logger->message(Tools::Log::MessageLevels::debug, "X size is " + std::to_string(x.size()));
				std::stringstream ss;
				for(auto &xx : x) {
					ss << xx << ", ";
				}
				logger->message(Tools::Log::MessageLevels::debug, ss.str());
			}
			{
				logger->message(Tools::Log::MessageLevels::debug, "Y size is " + std::to_string(y.size()));
				std::stringstream ss;
				for(auto &yy : y) {
					ss << yy << ", ";
				}
				logger->message(Tools::Log::MessageLevels::debug, ss.str());
			}
			x.clear(), y.clear();
		}
	}
	catch(const std::exception &error) {
		std::cerr << error.what() << std::endl;
	}
	return 0;
}

