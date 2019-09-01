
/**
  *
  * @file InverseForHumanoid.cpp
  * @author Naoki Takahashi
  *
  **/

#include "InverseForHumanoid.hpp"

#include <stdexcept>

namespace Kinematics {
	InverseForHumanoid::InverseForHumanoid() {
		body_orbit_map = std::make_unique<BodyOrbitMap>();
	}

	void InverseForHumanoid::load_model(ModelPtr &new_model) {
		if(new_model) {
			model = new_model;
		}
		else {
			throw std::runtime_error("Can not load model from Kinematics::InverseForHumanoid");
		}
	}

	void InverseForHumanoid::set_body_id(const ControlBodyPoint &point_identity, const BodyID &id) {
		control_map[point_identity].body_id = id;
	}

	InverseForHumanoid &InverseForHumanoid::head(const BodyID &id) {
		set_body_id(ControlBodyPoint::head, id);
		return *this;
	}

	InverseForHumanoid &InverseForHumanoid::left_low_arm(const BodyID &id) {
		set_body_id(ControlBodyPoint::left_low_arm, id);
		return *this;
	}

	InverseForHumanoid &InverseForHumanoid::left_tibia(const BodyID &id) {
		set_body_id(ControlBodyPoint::left_tibia, id);
		return *this;
	}

	InverseForHumanoid &InverseForHumanoid::left_ankle(const BodyID &id) {
		set_body_id(ControlBodyPoint::left_ankle, id);
		return *this;
	}

	InverseForHumanoid &InverseForHumanoid::right_low_arm(const BodyID &id) {
		set_body_id(ControlBodyPoint::right_low_arm, id);
		return *this;
	}

	InverseForHumanoid &InverseForHumanoid::right_tibia(const BodyID &id) {
		set_body_id(ControlBodyPoint::right_tibia, id);
		return *this;
	}

	InverseForHumanoid &InverseForHumanoid::right_ankle(const BodyID &id) {
		set_body_id(ControlBodyPoint::right_ankle, id);
		return *this;
	}

	bool InverseForHumanoid::has_control_body_point() {
		if(control_map.size() == number_of_body_point) {
			return true;
		}
		return false;
	}

	void InverseForHumanoid::update_control_orbits() {
	}
}

