
/**
  *
  * @file BodyOrbitMap.cpp
  * @author Naoki Takahashi
  *
  **/

#include "BodyOrbitMap.hpp"

#include <limits>

#include <Eigen/Geometry>

namespace Kinematics {
	BodyOrbitMap::BodyOrbitMap() {
	}

	bool BodyOrbitMap::has_body_id(const BodyID &body_id) {
		if(body_map.count(body_id) == 0) {
			return false;
		}
		return true;
	}

	void BodyOrbitMap::append_body_orbit(const BodyID &body_id, const Vector3 &central_point) {
		update_body_orbit(body_id, central_point);
	}

	void BodyOrbitMap::update_body_orbit(const BodyID &body_id, const Vector3 &central_point) {
		static BodyOrbit world_orbit,
						 local_orbit;

		long unsigned int i = 0;
		for(auto &&wo : world_orbit) {
			wo = central_point;
			local_orbit.at(i) = local_orbit.at(i).Zero();
			if(i != 0) {
				world_orbit.at(i)(i) += BodyTwinOrbit::shift_value;
				local_orbit.at(i)(i) += BodyTwinOrbit::shift_value;
			}
			i ++;
		}

		body_map[body_id].world_orbit = world_orbit;
		body_map[body_id].local_orbit = local_orbit;
	}

	void BodyOrbitMap::update_rotation_body_orbit(const BodyID &body_id, const Vector3 &euler_angle) {
		static Eigen::Quaternion<VectorElement> q;

		q = Eigen::AngleAxis<VectorElement>(euler_angle.z(), euler_angle.UnitZ());
		q = q * Eigen::AngleAxis<VectorElement>(euler_angle.y(), euler_angle.UnitY());
		q = q * Eigen::AngleAxis<VectorElement>(euler_angle.x(), euler_angle.UnitX());

		for(auto &&wo : body_map[body_id].world_orbit) {
			wo = q * wo;
		}
	}

	BodyOrbitMap::BodyOrbit BodyOrbitMap::get_world_orbit(const BodyID &body_id) {
		if(!has_body_id(body_id)) {
			throw std::runtime_error("Has not body id: " + std::to_string(body_id) + " from Kinematics::BodyOrbitMap");
		}
		return body_map[body_id].world_orbit;
	}

	BodyOrbitMap::BodyOrbit BodyOrbitMap::get_local_orbit(const BodyID &body_id) {
		if(!has_body_id(body_id)) {
			throw std::runtime_error("Has not body id: " + std::to_string(body_id) + " from Kinematics::BodyOrbitMap");
		}
		return body_map[body_id].local_orbit;
	}
}

