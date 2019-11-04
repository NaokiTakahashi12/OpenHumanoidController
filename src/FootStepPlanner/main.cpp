
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
		logger->message(Tools::Log::MessageLevels::debug, "Make pointer");

		FootStepPlanner::HumanoidFootprintManager<double>::Ptr footprint_manager = FootStepPlanner::HumanoidFootprintManager<double>::make_ptr();

		if(!footprint_manager) {
			throw std::runtime_error("Failed make footprint_manager");
		}
		logger->message(Tools::Log::MessageLevels::debug, "Success make pointer");

		footprint_manager->choice_footprint_planner("footstep_planner.conf.json");

		footprint_manager->set_begin(-0.2, 0.2, 0, 0, 0, 0);
		footprint_manager->set_goal(1, -0.4, 0, 0, 0, 0);

		footprint_manager->make_full_footprint();

		//! Footprint printing
		{
			std::vector<double> x, y;

			logger->message(Tools::Log::MessageLevels::debug, "Landing point line");

			for(auto &&fp : footprint_manager->get_footprint_list()) {
				x.push_back(fp.left.x()), x.push_back(fp.right.x());
				y.push_back(fp.left.y()), y.push_back(fp.right.y());
			}
			logger->message(Tools::Log::MessageLevels::debug, "X size is " + std::to_string(x.size()));
			logger->message(Tools::Log::MessageLevels::debug, "Y size is " + std::to_string(y.size()));

			{
				for(unsigned int i = 0; i < x.size(); i ++) {
					std::stringstream ss;

					ss << "ML:" << x.at(i) << "," << y.at(i);

					logger->message(Tools::Log::MessageLevels::debug, ss.str());
				}
			}
			x.clear(), y.clear();

			logger->message(Tools::Log::MessageLevels::debug, "Landing point left only");

			for(auto &&fp : footprint_manager->get_footprint_list()) {
				x.push_back(fp.left.x());
				y.push_back(fp.left.y());
			}
			logger->message(Tools::Log::MessageLevels::debug, "X size is " + std::to_string(x.size()));
			logger->message(Tools::Log::MessageLevels::debug, "Y size is " + std::to_string(y.size()));

			{
				for(unsigned int i = 0; i < x.size(); i ++) {
					std::stringstream ss;

					ss << "LL:" << x.at(i) << "," << y.at(i);

					logger->message(Tools::Log::MessageLevels::debug, ss.str());
				}
			}
			x.clear(), y.clear();

			logger->message(Tools::Log::MessageLevels::debug, "Landing point right only");

			for(auto &&fp : footprint_manager->get_footprint_list()) {
				x.push_back(fp.right.x());
				y.push_back(fp.right.y());
			}
			logger->message(Tools::Log::MessageLevels::debug, "X size is " + std::to_string(x.size()));
			logger->message(Tools::Log::MessageLevels::debug, "Y size is " + std::to_string(y.size()));

			{
				for(unsigned int i = 0; i < x.size(); i ++) {
					std::stringstream ss;

					ss << "RL:" << x.at(i) << "," << y.at(i);

					logger->message(Tools::Log::MessageLevels::debug, ss.str());
				}
			}
			x.clear(), y.clear();
		}
	}
	catch(const std::exception &error) {
		std::cerr << error.what() << std::endl;
	}
	return 0;
}

