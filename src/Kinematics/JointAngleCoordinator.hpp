
/**
  *
  * @file JointAngleCoordinator.hpp
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include <map>

#include "Quantity/JointAngle.hpp"
#include "Parameters.hpp"

namespace Kinematics {
	template <typename Scalar>
	class JointAngleCoordinator {
		public :
			using Ptr = std::unique_ptr<JointAngleCoordinator>;
			
			JointAngleCoordinator(typename Parameters<Scalar>::Ptr &);
			~JointAngleCoordinator();

			static Ptr make_ptr(typename Parameters<Scalar>::Ptr &);

			void set_config_file(const std::string &config_dir, const std::string &config_file_name);

			typename Quantity::JointAngle<Scalar>::VectorN offset_joint_angles();

		private :
			std::string config_dir,
						config_file_name;

			std::map<int, float> offset_joint_angle_map;

			typename Parameters<Scalar>::Ptr parameters;

			void initialize();
	};
}

