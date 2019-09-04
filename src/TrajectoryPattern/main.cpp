
#include <exception>
#include <iostream>

#include <Tools/Log/Logger.hpp>

#include "LinerInvertedPendulum.hpp"

int  main(int argc, char **argv) {
	Tools::Log::LoggerPtr logger;
	logger = std::make_shared<Tools::Log::Logger>(argc, argv);
	logger->start_loger_thread();

	try {
		TrajectoryPattern::LinerInvertedPendulum::FootPrintList footprint;

		footprint.push_back(TrajectoryPattern::LinerInvertedPendulum::Vector2(0, 0));
		footprint.push_back(TrajectoryPattern::LinerInvertedPendulum::Vector2(0, .05));

		footprint.push_back(TrajectoryPattern::LinerInvertedPendulum::Vector2(.15, .1));
		footprint.push_back(TrajectoryPattern::LinerInvertedPendulum::Vector2(.15, .1));
		footprint.push_back(TrajectoryPattern::LinerInvertedPendulum::Vector2(.15, .1));
		footprint.push_back(TrajectoryPattern::LinerInvertedPendulum::Vector2(.15, .1));
		footprint.push_back(TrajectoryPattern::LinerInvertedPendulum::Vector2(.15, .1));

		footprint.push_back(TrajectoryPattern::LinerInvertedPendulum::Vector2(0,  .05));
		footprint.push_back(TrajectoryPattern::LinerInvertedPendulum::Vector2(.0, .0));
		footprint.push_back(TrajectoryPattern::LinerInvertedPendulum::Vector2(.0, .0));
		footprint.push_back(TrajectoryPattern::LinerInvertedPendulum::Vector2(.0, .0));

		for(auto &&fp : footprint) {
			static auto i = 0;
			if(static_cast<bool>(i % 2)) {
				fp.y() *= -1;
			}
			i ++;
		}
		for(auto i = footprint.begin() + 1; i <= footprint.cend(); ++ i) {
			*i += *(i - 1);
		}

		TrajectoryPattern::LinerInvertedPendulum liner_inverted_pendulum;

		liner_inverted_pendulum.set_footprint_list(footprint);
		liner_inverted_pendulum.set_com_hight(1);

		liner_inverted_pendulum.compute(1, 0.02, 10, 1);

		for(auto &&fp : liner_inverted_pendulum.changed_footprint) {
			std::stringstream ss;
			ss << fp.transpose();
			logger->message(Tools::Log::MessageLevels::debug, ss.str());
		}
		{
			std::vector<double> x, y;
			for(auto &v : footprint) {
				x.push_back(v.x());
				y.push_back(v.y());
			}
		}
		{
			std::vector<double> x, y;
			for(auto &v : liner_inverted_pendulum.changed_footprint) {
				x.push_back(v.x());
				y.push_back(v.y());
			}
		}
		{
			std::vector<double> x, y;
			for(auto &v : liner_inverted_pendulum.com_line) {
				x.push_back(v.x());
				y.push_back(v.y());
			}
		}
	}
	catch(const std::exception &error) {
		std::cerr << error.what() << std::endl;
	}
	return 0;
};

