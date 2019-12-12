
#include "JointAngleModificator.hpp"

#include <cmath>
#include <stdexcept>

#include "ConfigManager.hpp"

namespace Kinematics {
	template <typename Scalar>
	JointAngleModificator<Scalar>::JointAngleModificator(typename Parameters<Scalar>::Ptr &parameters, const std::string &dir, const std::string &config_file) {
		this->parameters = parameters;

		auto config_manager = ConfigManager::make_ptr(dir, config_file);
		auto ids = config_manager->get_value_tree<int>("Joint limits", "ID");
		auto lower_limit = config_manager->get_value_tree<float>("Joint limits", "Lower");
		auto upper_limit = config_manager->get_value_tree<float>("Joint limits", "Upper");

		if(!(ids.size() == lower_limit.size() && ids.size() == upper_limit.size())) {
			throw std::runtime_error("Failed read joint limit from Kinematics::JointAngleModificator");
		}
		for(unsigned int i = 0; i < ids.size(); i ++) {
			limit_map[ids.at(i) - 1] = {lower_limit.at(i), upper_limit.at(i)};
		}
	}

	template <typename Scalar>
	JointAngleModificator<Scalar>::~JointAngleModificator() {
	}

	template <typename Scalar>
	typename JointAngleModificator<Scalar>::Ptr JointAngleModificator<Scalar>::make_ptr(typename Parameters<Scalar>::Ptr &parameters, const std::string &dir, const std::string &config_file) {
		return std::make_unique<JointAngleModificator>(parameters, dir, config_file);
	}

	template <typename Scalar>
	void JointAngleModificator<Scalar>::override_round_function(OverrideRoundFunction function) {
		if(!function) {
			throw std::runtime_error("Faild entry override round function from Kinematics::JointAngleModificator");
		}

		round_function = function;
	}

	template <typename Scalar>
	void JointAngleModificator<Scalar>::assert_round() {
		if(!round()) {
			throw std::runtime_error("Failed joint angle round from Kinematics::JointAngleModificator");
		}
	}

	template <typename Scalar>
	bool JointAngleModificator<Scalar>::round() {
		if(round_function) {
			return round_function(parameters->joint_angle());
		}

		for(auto i = 0; i < parameters->joint_angle().position().size(); i ++) {
			constexpr auto threshold = 2 * M_PI;

			Scalar angle = parameters->joint_angle().position()(i);
			const Scalar sign = angle / std::abs(angle);

			while(threshold <= std::abs(angle)) {
				angle -= sign * threshold;
			}
			const auto a = angle;
			const auto b = threshold - std::abs(a);

			if(std::abs(a) < std::abs(b)) {
				parameters->joint_angle().position()(i) = a;
			}
			else {
				parameters->joint_angle().position()(i) = b;
			}
		}

		return true;
	}

	template <typename Scalar>
	void JointAngleModificator<Scalar>::assert_limit_angle() {
		if(is_limit_angle()) {
			throw std::runtime_error("Failed over joint angle from Kinematics::JointAngleModificator");
		}
	}

	template <typename Scalar>
	bool JointAngleModificator<Scalar>::is_limit_angle() {
		if(parameters->joint_angle()().size() != static_cast<unsigned int>(limit_map.size())) {
			throw std::runtime_error("Failed different size joint angle from Kinematics::JointAngleModificator");
		}
		for(unsigned int i = 0; i < limit_map.size(); i ++) {
			const auto lower = limit_map[i].first;
			const auto upper = limit_map[i].second;

			if(lower > parameters->joint_angle()()(i) && lower != 0) {
				return true;
			}
			if(upper < parameters->joint_angle()()(i) && upper != 0) {
				return true;
			}
		}

		return false;
	}

	template <typename Scalar>
	void JointAngleModificator<Scalar>::assert_modify() {
		assert_round();
		assert_limit_angle();
	}

	template <typename Scalar>
	bool JointAngleModificator<Scalar>::modify() {
		const auto is_round = !round();
		const auto is_limit = is_limit_angle();

		return !is_round && !is_limit;
	}

	template class JointAngleModificator<float>;
	template class JointAngleModificator<double>;
}

