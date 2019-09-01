
/**
  *
  * @file BodyOrbitMap.hpp
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include <array>
#include <string>
#include <stdexcept>
#include <unordered_map>

#include <Tools/Math/Matrix.hpp>

#include "Model.hpp"

namespace Kinematics {
	class BodyOrbitMap {
		public :
			using VectorElement = Model::VectorNElement;
			using Vector3 = Tools::Math::Vector3<VectorElement>;
			using BodyID = Model::BodyID;

			//! Centor point = 0, X shift = 1, Y shift = 2
			using BodyOrbit = std::array<Vector3, 3>;

			BodyOrbitMap();

			bool has_body_id(const BodyID &);

			void append_body_orbit(const BodyID &body_id, const Vector3 &central_point);
			void update_body_orbit(const BodyID &body_id, const Vector3 &central_point);

			void update_rotation_body_orbit(const BodyID &body_id, const Vector3 &euler_angle);

			BodyOrbit get_world_orbit(const BodyID &),
					  get_local_orbit(const BodyID &);

		private :
			struct BodyTwinOrbit {
				static constexpr VectorElement shift_value = 1;
				BodyOrbit world_orbit,
						  local_orbit;
			};

			using BodyMap = std::unordered_map<BodyID, BodyTwinOrbit>;

			BodyMap body_map;

	};
}

