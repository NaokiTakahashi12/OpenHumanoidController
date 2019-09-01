
/**
  *
  * @file InverseForHumanoid.hpp
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include <vector>
#include <unordered_map>

#include "Model.hpp"
#include "BodyOrbitMap.hpp"

namespace Kinematics {
	class InverseForHumanoid {
		public :
			using Vector3 = BodyOrbitMap::Vector3;
			using BodyID = Model::BodyID;

			InverseForHumanoid();

			void load_model(ModelPtr &);

			enum class ControlBodyPoint {
				head,
				left_low_arm,
				left_tibia,
				left_ankle,
				right_low_arm,
				right_tibia,
				right_ankle
			};
			static constexpr unsigned int number_of_body_point = 7;

			struct ControlBodyParameter {
				BodyID body_id;
				Vector3 default_body_position;
			};

			void set_body_id(const ControlBodyPoint &, const BodyID &);

			InverseForHumanoid &head(const BodyID &),
							   &left_low_arm(const BodyID &),
							   &left_tibia(const BodyID &),
							   &left_ankle(const BodyID &),
							   &right_low_arm(const BodyID &),
							   &right_tibia(const BodyID &),
							   &right_ankle(const BodyID &);

		private :
			ModelPtr model;

			std::unique_ptr<BodyOrbitMap> body_orbit_map;

			std::unordered_map<ControlBodyPoint, ControlBodyParameter> control_map;

			bool has_control_body_point();
			void update_control_orbits();

	};
}

