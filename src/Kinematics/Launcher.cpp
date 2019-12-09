
/**
  *
  * @file Launcher.hpp
  * @author Naoki Takahashi
  *
  **/

#include "Launcher.hpp"

#include <iostream>
#include <chrono>

#include "Model/Humanoid.hpp"
#include "ConfigManager.hpp"

namespace Kinematics {
	template <typename Scalar>
	Launcher<Scalar>::Launcher(RobotStatus::InformationPtr &robot_status_information_ptr, const std::string &config_dir, const std::string &config_file) : config_dir(config_dir), config_file(config_file) {
		auto config_manager = ConfigManager::make_ptr(config_dir, config_file);

		robo_info = robot_status_information_ptr;

		make_model(config_manager->get_value<std::string>("Model.Directory") + config_manager->get_value<std::string>("Model.File"));

		parameters = Parameters<Scalar>::make_ptr(model->dof());
		control_point_map = ControlPointMap<Scalar>::make_ptr();
		set_control_point_from_config_for_humanoid(config_manager->get_value<std::string>("Control point map config"));

		solver_manager = SolverManager<Scalar>::make_ptr(model, config_dir, config_file);
		joint_angle_modificator = JointAngleModificator<Scalar>::make_ptr(parameters, config_dir, config_manager->get_value<std::string>("Joint space config"));
	}

	template <typename Scalar>
	Launcher<Scalar>::~Launcher() {
		close();
	}

	template <typename Scalar>
	void Launcher<Scalar>::initialize() {
		solver_manager->entry_parameters(parameters);
		solver_manager->entry_control_point_map(control_point_map);

		solver_manager->spawn_solver();
	}

	template <typename Scalar>
	void Launcher<Scalar>::close() {
		enable_loop = false;
		if(runtime_thread) {
			runtime_thread->join();
		}
	}

	template <typename Scalar>
	void Launcher<Scalar>::entry_new_fk_solver(FKObject fk_object) {
		solver_manager->entry_new_fk_solver(std::move(fk_object));
	}

	template <typename Scalar>
	void Launcher<Scalar>::entry_new_ik_solver(IKObject ik_object) {
		solver_manager->entry_new_ik_solver(std::move(ik_object));
	}

	template <typename Scalar>
	typename Parameters<Scalar>::Ptr Launcher<Scalar>::get_parameters() {
		return parameters;
	}

	template <typename Scalar>
	typename ControlPointMap<Scalar>::Ptr Launcher<Scalar>::get_control_point_map() {
		return control_point_map;
	}

	template <typename Scalar>
	Model::RBDLBased::Ptr Launcher<Scalar>::get_model() {
		return model;
	}

	template <typename Scalar>
	std::mutex &Launcher<Scalar>::get_mutex() {
		return access_mutex;
	}

	template <typename Scalar>
	void Launcher<Scalar>::operator () () {
		if(!solver_manager->is_ready()) {
			throw std::runtime_error("Could not launch solver_manager from Kinematics::Launcher");
		}

		enable_loop = true;
		solver_manager->fk();
		solver_manager->first_ik();

		runtime_thread = std::make_unique<std::thread>(
			[this]() {
			static auto cache = control_point_map->get_list();

				while(enable_loop) {
					const auto lock = std::lock_guard<std::mutex>(access_mutex);

					if(is_update(cache)) {
						if(!solver_manager->ik()) {
							std::stringstream ss;
							for(auto &&[key, value] : control_point_map->access_to_this_storage()) {
								ss << "Key: " << key << std::endl;
								ss << "Point: " << value.point().transpose() << std::endl;
								ss << "Angle: " << value.angle().transpose() << std::endl;
								ss << std::endl;
							}
							throw std::runtime_error("Failed IK\n" + ss.str());
						}

						joint_angle_modificator->modify();

						solver_manager->fk();
					}
					else {
						std::this_thread::yield();
					}
				}
			}
		);
	}

	template <typename Scalar>
	void Launcher<Scalar>::make_model(const std::string &model_file) {
		if(model) {
			model.reset();
		}

		model = Model::Humanoid::make_ptr_from_urdf(model_file);

		if(!model) {
			throw std::runtime_error("Failed make model from Kinematics::Launcher");
		}

		model->initialize();
	}

	template <typename Scalar>
	void Launcher<Scalar>::set_control_point_from_config_for_humanoid(const std::string &config_file) {
		auto config_manager = ConfigManager::make_ptr(config_dir, config_file);

		control_point_map->set(
			model->body_id(
				config_manager->template get_value<std::string>("Control point names.Left arm")
			)
		);
		control_point_map->set(
			model->body_id(
				config_manager->template get_value<std::string>("Control point names.Right arm")
			)
		);
		control_point_map->set(
			model->body_id(
				config_manager->template get_value<std::string>("Control point names.Left foot")
			)
		);
		control_point_map->set(
			model->body_id(
				config_manager->template get_value<std::string>("Control point names.Right foot")
			)
		);
		control_point_map->set(
			model->body_id(
				config_manager->template get_value<std::string>("Control point names.Head")
			)
		);
	}

	template <typename Scalar>
	bool Launcher<Scalar>::is_update(typename ControlPointMap<Scalar>::DataList &cache) {
		const auto current = control_point_map->get_list();

		for(unsigned int i = 0; i < cache.size(); i ++) {
			if(cache.at(i) != current.at(i)) {
				cache = current;
				return true;
			}
		}

		return false;
	}

	template class Launcher<float>;
	template class Launcher<double>;
}

