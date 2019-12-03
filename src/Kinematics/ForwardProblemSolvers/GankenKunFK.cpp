
/**
  *
  * @file GankenKunFK.hpp
  * @author Yasuo Hayashibara
  *
  **/

#include "GankenKunFK.hpp"

#include <rbdl/Kinematics.h>

namespace Kinematics {
	namespace ForwardProblemSolvers {
		template <typename Scalar>
		GankenKunFK<Scalar>::GankenKunFK(ModelPtr &new_model) : MultipleFK<Scalar>(new_model) {
		}

		template <typename Scalar>
		GankenKunFK<Scalar>::~GankenKunFK() {
		}

		template <typename Scalar>
		const std::string GankenKunFK<Scalar>::get_key() {
			return "GankenKunFK";
		}

		template <typename Scalar>
		typename GankenKunFK<Scalar>::Ptr GankenKunFK<Scalar>::make_ptr(ModelPtr &new_model) {
			return std::make_unique<GankenKunFK>(new_model);
		}

		template <>
		bool GankenKunFK<double>::compute() {
			const auto base_position = Quantity::SpatialPoint<double>::Vector3::Zero();
			for(auto &&[body_id, spatial_point] : this->control_point_map->access_to_this_storage()) {
				spatial_point.point(
					RigidBodyDynamics::CalcBodyToBaseCoordinates(
						this->model->access(),
						this->parameters->joint_angle()(),
						body_id,
						base_position,
						true
					)
				)
				.rotation(
					RigidBodyDynamics::CalcBodyWorldOrientation(
						this->model->access(),
						this->parameters->joint_angle()(),
						body_id,
						false
					)
				);
			}
			return true;
		}

		template <>
		[[deprecated("Please use double template.")]]
		bool GankenKunFK<float>::compute() {
			const auto base_position = Quantity::SpatialPoint<double>::Vector3::Zero();

			for(auto &&[body_id, spatial_point] : this->control_point_map->access_to_this_storage()) {
				spatial_point.point(
					RigidBodyDynamics::CalcBodyToBaseCoordinates(
						this->model->access(),
						this->parameters->joint_angle()().cast<double>(),
						body_id,
						base_position,
						true
					).cast<float>()
				)
				.rotation(
					RigidBodyDynamics::CalcBodyWorldOrientation(
						this->model->access(),
						this->parameters->joint_angle()().cast<double>(),
						body_id,
						false
					).cast<float>()
				);
			}
			return true;
		}

		template class GankenKunFK<float>;
		template class GankenKunFK<double>;
	}
}

