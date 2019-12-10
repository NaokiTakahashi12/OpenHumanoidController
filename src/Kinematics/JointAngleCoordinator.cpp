
/**
  *
  * @file JointAngleCoordinator.cpp
  * @author Naoki Takahashi
  *
  **/

#include "JointAngleCoordinator.hpp"

#include "ConfigManager.hpp"

namespace Kinematics {
	template <typename Scalar>
	JointAngleCoordinator<Scalar>::JointAngleCoordinator(typename Parameters<Scalar>::Ptr &new_parameters) {
		parameters = new_parameters;
	}

	template <typename Scalar>
	JointAngleCoordinator<Scalar>::~JointAngleCoordinator() {
	}

	template <typename Scalar>
	typename JointAngleCoordinator<Scalar>::Ptr JointAngleCoordinator<Scalar>::make_ptr(typename Parameters<Scalar>::Ptr &new_parameters) {
		return std::make_unique<JointAngleCoordinator<Scalar>>(new_parameters);
	}

	template <typename Scalar>
	void JointAngleCoordinator<Scalar>::set_config_file(const std::string &config_dir,  const std::string &config_file_name) {
		this->config_dir = config_dir;
		this->config_file_name = config_file_name;

		initialize();
	}

	template <typename Scalar>
	typename Quantity::JointAngle<Scalar>::VectorN JointAngleCoordinator<Scalar>::offset_joint_angles() {
		if(static_cast<unsigned int>(parameters->joint_angle().position().size()) != offset_joint_angle_map.size()) {
			throw std::runtime_error(
				"Failed different size of parameter.joint_angle("
				+ std::to_string(parameters->joint_angle().position().size())
				+ ") vs offset_joint_angle_map("
				+ std::to_string(offset_joint_angle_map.size())
				+ ") from Kinematics::JointAngleCoordinator");
		}
		auto ja = parameters->joint_angle().position();

		for(unsigned int i = 0; i < ja.size(); i ++) {
			ja(i) += offset_joint_angle_map[i + 1];
		}

		return ja;
	}

	template <typename Scalar>
	void JointAngleCoordinator<Scalar>::initialize() {
		ConfigManager config(config_dir, config_file_name);
		
		auto ids = config.get_value_tree<int>("Joint offset", "ID");
		auto angles = config.get_value_tree<float>("Joint offset", "Angle");

		if(ids.size() != angles.size()) {
			throw std::runtime_error("Failed different size of ID vs Angle for Kinematics::JointAngleCoordinator");
		}
		for(unsigned int i = 0; i < ids.size(); i ++) {
			offset_joint_angle_map[ids.at(i)] = angles.at(i);
		}
		if(ids.size() != offset_joint_angle_map.size()) {
			throw std::runtime_error("Failed different size of ID vs offset_joint_angle_map for Kinematics::JointAngleCoordinator");
		}
	}

	template class JointAngleCoordinator<float>;
	template class JointAngleCoordinator<double>;
}

