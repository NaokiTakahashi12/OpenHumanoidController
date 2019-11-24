
/**
  *
  * @file SimpleForSerialLink.hpp
  * @author Naoki Takahashi
  *
  **/

#include "SimpleForSerialLink.hpp"

#include <rbdl/Kinematics.h>

namespace Kinematics {
	namespace ForwardProblemSolvers {
		template <typename Scalar>
		SimpleForSerialLink<Scalar>::SimpleForSerialLink(ModelPtr &new_model) : MultipleFK<Scalar>(new_model) {
		}

		template <typename Scalar>
		SimpleForSerialLink<Scalar>::~SimpleForSerialLink() {
		}

		template <typename Scalar>
		const std::string SimpleForSerialLink<Scalar>::get_key() {
			return "SimpleForSerialLink";
		}

		template <typename Scalar>
		typename SimpleForSerialLink<Scalar>::Ptr SimpleForSerialLink<Scalar>::make_ptr(ModelPtr &new_model) {
			return std::make_unique<SimpleForSerialLink>(new_model);
		}

		template <>
		bool SimpleForSerialLink<double>::compute() {
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
		bool SimpleForSerialLink<float>::compute() {
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

		template class SimpleForSerialLink<float>;
		template class SimpleForSerialLink<double>;
	}
}

