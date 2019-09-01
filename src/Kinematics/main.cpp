
#include <iostream>
#include <exception>
#include <sstream>
#include <vector>
#include <string>

#include <Tools/Log/Logger.hpp>

#include "Model.hpp"
#include "BodyOrbitMap.hpp"

int main(int argc, char **argv) {
	Tools::Log::LoggerPtr logger;
	logger = std::make_shared<Tools::Log::Logger>(argc, argv);
	logger->start_loger_thread();

	try {
		std::string urdf_filename;
		urdf_filename = "../../../data/urdf/DARwInOP.urdf";

		Kinematics::ModelPtr model;
		model = std::make_shared<Kinematics::Model>(urdf_filename);

		if(model->is_load_model()) {
			logger->message(Tools::Log::MessageLevels::info, "Model load success");
		}
		else {
			logger->message(Tools::Log::MessageLevels::info, "Model load fatal");
		}

		RigidBodyDynamics::InverseDynamics(model->access(), model->get_q(), model->get_qd(), model->get_qdd(), model->access_tau());
		RigidBodyDynamics::ForwardDynamics(model->access(), model->get_q(), model->get_qd(), model->get_tau(), model->access_qdd());
		RigidBodyDynamics::UpdateKinematics(model->access(), model->get_q(), model->get_qd(), model->get_qdd());

		Kinematics::BodyOrbitMap coordinate_base;
		Kinematics::BodyOrbitMap::Vector3 body_centor;
		Kinematics::Model::BodyIdentity body_name("MP_ANKLE2_R");
		const auto body_id = model->get_body_id(body_name);

		{
			body_centor = RigidBodyDynamics::CalcBaseToBodyCoordinates(model->access(), model->get_q(), body_id, body_centor.Zero());
			auto swap_body_centor = 0.89 * body_centor;
			body_centor.x() = swap_body_centor.z();
			body_centor.y() = swap_body_centor.y();
			body_centor.z() = -1 * swap_body_centor.x();

			coordinate_base.append_body_orbit(body_id, body_centor);

			logger->message(Tools::Log::MessageLevels::debug, body_name + " Body ID is " + std::to_string(body_id));
		}

		auto orbit = coordinate_base.get_world_orbit(body_id);

		std::vector<Kinematics::BodyOrbitMap::BodyID> body_ids;
		body_ids.push_back(body_id);
		body_ids.push_back(body_id);
		body_ids.push_back(body_id);
		std::vector<RigidBodyDynamics::Math::Vector3d> body_point;
		body_point.push_back(Kinematics::BodyOrbitMap::Vector3(0, 0, 0));
		body_point.push_back(Kinematics::BodyOrbitMap::Vector3(0, 1, 0));
		body_point.push_back(Kinematics::BodyOrbitMap::Vector3(0, 0, 1));

		std::vector<RigidBodyDynamics::Math::Vector3d> target_point;
		for(auto &o : orbit) {
			std::stringstream ss;
			ss << "Body orbit " << o.transpose();
			logger->message(Tools::Log::MessageLevels::debug, ss.str());
			target_point.push_back(o);
		}

		auto q_result = model->get_q();

		auto is_success = RigidBodyDynamics::InverseKinematics(
				model->access(),
				model->get_q(),
				body_ids,
				body_point,
				target_point,
				q_result
		);
		q_result *= M_PI / 180;
		if(is_success) {
			logger->message(Tools::Log::MessageLevels::debug, "Success");
			std::stringstream ss;
			ss << q_result.transpose();
			logger->message(Tools::Log::MessageLevels::debug, ss.str());
		}
		else {
			logger->message(Tools::Log::MessageLevels::debug, "Fatal");
			std::stringstream ss;
			ss << q_result.transpose();
			logger->message(Tools::Log::MessageLevels::debug, ss.str());
		}

		{
			logger->message(Tools::Log::MessageLevels::debug, "Q");
			std::stringstream ss;
			ss << model->get_q().transpose();
			logger->message(Tools::Log::MessageLevels::debug, ss.str());
		}
		{
			logger->message(Tools::Log::MessageLevels::debug, "Qd");
			std::stringstream ss;
			ss << model->get_qd().transpose();
			logger->message(Tools::Log::MessageLevels::debug, ss.str());
		}
		{
			logger->message(Tools::Log::MessageLevels::debug, "Qdd");
			std::stringstream ss;
			ss << model->get_qdd().transpose();
			logger->message(Tools::Log::MessageLevels::debug, ss.str());
		}
		{
			logger->message(Tools::Log::MessageLevels::debug, "Tau");
			std::stringstream ss;
			ss << model->get_tau().transpose();
			logger->message(Tools::Log::MessageLevels::debug, ss.str());
		}

	}
	catch(const std::exception &error) {
		std::cerr << error.what() << std::endl;
		logger->message(Tools::Log::MessageLevels::fatal, error.what());
	}

	return 0;
}

