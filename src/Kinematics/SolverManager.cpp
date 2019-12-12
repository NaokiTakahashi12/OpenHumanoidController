
/**
  *
  * @file SolverManager.cpp
  * @author Naoki Takahashi
  *
  **/

#include "SolverManager.hpp"

#include <stdexcept>
#include <map>

#include "ConfigManager.hpp"

namespace Kinematics {
	template <typename Scalar>
	SolverManager<Scalar>::SolverManager(ModelPtr &new_model, const std::string &dir, const std::string &filename) : config_dir(dir), config_filename(filename) {
		fk_selector = FKSelector::make_ptr(new_model);
		ik_selector = IKSelector::make_ptr(new_model);
	}

	template <typename Scalar>
	SolverManager<Scalar>::~SolverManager() {
	}

	template <typename Scalar>
	typename SolverManager<Scalar>::Ptr SolverManager<Scalar>::make_ptr(ModelPtr &new_model, const std::string &dir, const std::string &name) {
		return std::make_unique<SolverManager>(new_model, dir, name);
	}

	template <typename Scalar>
	SolverManager<Scalar> &SolverManager<Scalar>::entry_new_fk_solver(typename FKSelector::EntryObject entry_object) {
		if(!fk_selector) {
			throw std::runtime_error("Failed can not access FK selector from Kinematics::SolverManager");
		}

		fk_selector->re_register_object(std::move(entry_object));

		return *this;
	}

	template <typename Scalar>
	SolverManager<Scalar> &SolverManager<Scalar>::entry_new_ik_solver(typename IKSelector::EntryObject entry_object) {
		if(!ik_selector) {
			throw std::runtime_error("Failed can not access IK selector from Kinematics::SolverManager");
		}

		ik_selector->re_register_object(std::move(entry_object));

		return *this;
	}

	template <typename Scalar>
	SolverManager<Scalar> &SolverManager<Scalar>::entry_parameters(typename Parameters<Scalar>::Ptr &param) {
		if(!param) {
			throw std::runtime_error("Parameters failed to enter from Kinematics::SolverManager");
		}

		parameters = param;

		return *this;
	}

	template <typename Scalar>
	SolverManager<Scalar> &SolverManager<Scalar>::entry_control_point_map(typename ControlPointMap<Scalar>::Ptr &map) {
		if(!map) {
			throw std::runtime_error("ControlPointMap failed to enter from Kinematics::SolverManager");
		}

		control_point_map = map;

		return *this;
	}

	template <typename Scalar>
	void SolverManager<Scalar>::spawn_solver() {
		if(!parameters) {
			throw std::runtime_error("Failed access to parameters pointer from Kinematics::SolverManager");
		}
		if(!control_point_map) {
			throw std::runtime_error("Failed access to control point map pointer from Kinematics::SolverManager");
		}

		select_solver_from_config(config_dir, config_filename);
		{
			auto config_manager = ConfigManager::make_ptr(config_dir, config_filename);
			entry_ik_joint_angle_from_config(config_dir, config_manager->template get_value<std::string>("Joint space config"));
		}

		fk_solver->register_parameters(parameters);
		ik_solver->register_parameters(parameters);

		fk_solver->register_map(control_point_map);
		ik_solver->register_map(control_point_map);

	}

	template <typename Scalar>
	bool SolverManager<Scalar>::is_ready_fk() {
		if(fk_solver) {
			return true;
		}
		
		return false;
	}

	template <typename Scalar>
	bool SolverManager<Scalar>::is_ready_ik() {
		if(ik_solver) {
			return true;
		}

		return false;
	}

	template <typename Scalar>
	bool SolverManager<Scalar>::is_ready() {
		return is_ready_fk() && is_ready_ik();
	}

	template <typename Scalar>
	bool SolverManager<Scalar>::fk() {
		if(!is_ready_fk()) {
			throw std::runtime_error("Failed access to FK solver from Kinematics::SolverManager");
		}

		return fk_solver->compute();
	}

	template <typename Scalar>
	bool SolverManager<Scalar>::first_ik() {
		if(!is_ready_ik()) {
			throw std::runtime_error("Failed access to IK solver from Kinematics::SolverManager");
		}

		if(before_ik_joint_angle) {
			return ik_solver->compute(*before_ik_joint_angle);
		}

		return ik_solver->compute();
	}

	template <typename Scalar>
	bool SolverManager<Scalar>::ik() {
		if(!is_ready_ik()) {
			throw std::runtime_error("Failed access to IK solver from Kinematics::SolverManager");
		}

		return ik_solver->compute();
	}

	template <typename Scalar>
	void SolverManager<Scalar>::select_solver_from_config(const std::string &dir, const std::string &filename) {
		auto config_manager = ConfigManager::make_ptr(dir, filename);

		select_solver_from_names(
			{
				config_manager->get_value<std::string>("Assign solver.Forward"),
				config_manager->get_value<std::string>("Assign solver.Inverse")
			}
		);
	}

	template <typename Scalar>
	void SolverManager<Scalar>::select_solver_from_names(const SolverNames solver_names) {
		fk_solver_selection(std::get<0>(solver_names));
		ik_solver_selection(std::get<1>(solver_names));
	}

	template <typename Scalar>
	void SolverManager<Scalar>::entry_ik_joint_angle_from_config(const std::string &dir, const std::string &filename) {
		auto config_manager = ConfigManager::make_ptr(dir, filename);

		std::map<int, float> angle_map;

		{
			const auto ids = config_manager->get_value_tree<float>("Before ik joint angle", "ID");
			const auto angles = config_manager->get_value_tree<float>("Before ik joint angle", "Angle");

			if(ids.size() != *std::max_element(ids.cbegin(), ids.cend())) {
				throw std::runtime_error("Different ID size from Kinematics::SolverManager");
			}
			if(ids.size() != angles.size()) {
				throw std::runtime_error("Different size of before ik joint angle from Kinematics::SolverManager");
			}

			for(unsigned int i = 0; i < ids.size(); i ++) {
				angle_map[ids.at(i) - 1] = angles.at(i);
			}
		}

		auto joint_angle = std::make_unique<Quantity::JointAngle<Scalar>>(angle_map.size());

		for(unsigned int i = 0; i < angle_map.size(); i ++) {
			joint_angle->position()(i) = angle_map[i];
		}

		entry_ik_joint_angle(std::move(joint_angle));
	}

	template <typename Scalar>
	void SolverManager<Scalar>::entry_ik_joint_angle(BeforeIKJointAngle ik_joint_angle) {
		if(!parameters) {
			throw std::runtime_error("Failed access parameters from Kinematics::SolverManager");
		}
		if(parameters->joint_angle().dof() != ik_joint_angle->dof()) {
			throw std::runtime_error("Different size of ik_joint_angle from Kinematics::SolverManager");
		}

		before_ik_joint_angle = std::move(ik_joint_angle);
	}

	template <typename Scalar>
	void SolverManager<Scalar>::fk_solver_selection(const std::string &name) {
		fk_solver = fk_selector->choice_object(name);

		if(!is_ready_fk()) {
			throw std::runtime_error("Failed select forward solver from Kinematics::SolverManager");
		}

		fk_selector.reset();
	}

	template <typename Scalar>
	void SolverManager<Scalar>::ik_solver_selection(const std::string &name) {
		ik_solver = ik_selector->choice_object(name);

		if(!is_ready_ik()) {
			throw std::runtime_error("Failed select inverse solver from Kinematics::SolverManager");
		}

		ik_selector.reset();
	}

	template class SolverManager<float>;
	template class SolverManager<double>;
}

