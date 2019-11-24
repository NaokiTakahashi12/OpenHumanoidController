
#include <iostream>
#include <exception>
#include <sstream>
#include <chrono>

#include <RobotStatus/Information.hpp>

#include "Launcher.hpp"
#include "InverseProblemSolvers/CustomMultipleIK.hpp"
#include "ForwardProblemSolvers/CustomMultipleFK.hpp"

void test_spatial_point(Tools::Log::LoggerPtr &);

int main(int argc, char **argv) {
	auto robo_info = std::make_shared<RobotStatus::Information>(argc, argv);
	auto logger = robo_info->logger;

	logger->start_loger_thread();

	try {
		logger->message(Tools::Log::MessageLevels::info, "Startup");

		Kinematics::Launcher<double> launcher("./", "kinematics.conf.json");
		logger->message(Tools::Log::MessageLevels::info, "Load config");

			auto model = launcher.get_model();
			auto control_point_map = launcher.get_control_point_map();
			logger->message(Tools::Log::MessageLevels::info, "Cloning control point map");
			auto parameters = launcher.get_parameters();
			logger->message(Tools::Log::MessageLevels::info, "Cloning parameters");

		{
			auto custom_fk = Kinematics::ForwardProblemSolvers::CustomMultipleFK<double>::make_ptr(model);
			custom_fk->register_map(control_point_map);
			custom_fk->register_parameters(parameters);
			custom_fk->register_solver_function(
				[](Kinematics::Model::RBDLBased::Ptr &, typename Kinematics::Parameters<double>::Ptr &, typename Kinematics::ControlPointMap<double>::Ptr &) {
					return true;
				}
			);
			launcher.entry_new_fk_solver(
				{custom_fk->get_key(), std::move(custom_fk)}
			);
		}
		{
			auto custom_ik = Kinematics::InverseProblemSolvers::CustomMultipleIK<double>::make_ptr(model);
			custom_ik->register_map(control_point_map);
			custom_ik->register_parameters(parameters);
			custom_ik->register_solver_function(
				[](Kinematics::Model::RBDLBased::Ptr &, typename Kinematics::Parameters<double>::Ptr &, typename Kinematics::ControlPointMap<double>::Ptr &) {
					return true;
				}
			);
			launcher.entry_new_ik_solver(
				{custom_ik->get_key(), std::move(custom_ik)}
			);
		}

		launcher.initialize();
		logger->message(Tools::Log::MessageLevels::info, "Launcher initialized");
		logger->message(Tools::Log::MessageLevels::info, "Success startup");

		launcher();
		logger->message(Tools::Log::MessageLevels::info, "Thread start");

		control_point_map->add(20, Kinematics::Quantity::SpatialPoint<double>().point(-0.001, 0.001, 0.1));
		control_point_map->add(14, Kinematics::Quantity::SpatialPoint<double>().point(-0.001, 0.001, 0.1));
		control_point_map->add(6, Kinematics::Quantity::SpatialPoint<double>().point(0, 0, -1e-4));
		control_point_map->add(3, Kinematics::Quantity::SpatialPoint<double>().point(0, 0, -1e-4));

		int i = 0;
		while(1) {
			const auto lock = std::lock_guard<std::mutex>(launcher.get_mutex());

			if(i == 1900) {
				logger->message(Tools::Log::MessageLevels::info, "Thread finish");
				break;
			}

			control_point_map->add(20, Kinematics::Quantity::SpatialPoint<double>().point(0, 0, -0.00005));
			control_point_map->add(14, Kinematics::Quantity::SpatialPoint<double>().point(0, 0, -0.00005));

			for(auto &&[body_id, spatial_point] : control_point_map->get_list_with_id()) {
				std::stringstream ss;
				ss << "Point of " << body_id << " : " << spatial_point.point().transpose();
				logger->message(Tools::Log::MessageLevels::trace, ss.str());
			}
			for(auto i = 0; i < parameters->joint_angle()().size(); i ++) {
				logger->message(Tools::Log::MessageLevels::trace, std::to_string(parameters->joint_angle()()(i)));
			}

			i ++;
		}
	}
	catch(const std::exception &error) {
		std::cerr << error.what() << std::endl;
	}

	logger->close_loger_thread();

	return 0;
}

void test_spatial_point(Tools::Log::LoggerPtr &logger) {
	Kinematics::Quantity::SpatialPoint spatial_point;
	{
		std::stringstream ss;
		ss << "Initial point of " << spatial_point.point().transpose();
		logger->message(Tools::Log::MessageLevels::debug, ss.str());
	}
	{
		std::stringstream ss;
		ss << "Initial Angle of " << spatial_point.angle().transpose();
		logger->message(Tools::Log::MessageLevels::debug, ss.str());
	}
	{
		std::stringstream ss;
		spatial_point.point(1, 2, 3);
		ss << "Added point(1, 2, 3) of " << spatial_point.point().transpose();
		logger->message(Tools::Log::MessageLevels::debug, ss.str());
	}
	{
		std::stringstream ss;
		spatial_point.angle(M_PI / 2, M_PI / 2, M_PI / 2);
		ss << "Added Angle(pi/2, pi/2, pi/2) of " << spatial_point.angle().transpose();
		logger->message(Tools::Log::MessageLevels::debug, ss.str());
	}
	{
		logger->message(Tools::Log::MessageLevels::debug, "Reconstruction test for rotation matrix");
		spatial_point.rotation(spatial_point.rotation());
		logger->message(Tools::Log::MessageLevels::debug, "Success reconstruction test for rotation matrix");
	}
	{
		std::stringstream ss;
		spatial_point += Kinematics::Quantity::SpatialPoint().point(-1, -2, -3);
		ss << "Added point(-1, -2, -3) of " << spatial_point.point().transpose();
		logger->message(Tools::Log::MessageLevels::debug, ss.str());
	}
	{
		std::stringstream ss;
		spatial_point += Kinematics::Quantity::SpatialPoint().angle(-M_PI / 2, -M_PI / 2, -M_PI / 2);
		ss << "Added Angle(-pi/2, -pi/2, -pi/2) of " << spatial_point.angle().transpose();
		logger->message(Tools::Log::MessageLevels::debug, ss.str());
	}
}

