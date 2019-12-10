
/**
  *
  * @file LevenbergMarquardtForSerialLink.cpp
  * @author Naoki Takahashi
  *
  **/

#include "LevenbergMarquardtForSerialLink.hpp"

#include <vector>
#include <iostream>

#include <rbdl/Kinematics.h>

namespace Kinematics {
	namespace InverseProblemSolvers {
		template <typename Scalar>
		LevenbergMarquardtForSerialLink<Scalar>::LevenbergMarquardtForSerialLink(ModelPtr &model_ptr) : MultipleIK<Scalar>(model_ptr) {
		}

		template <typename Scalar>
		LevenbergMarquardtForSerialLink<Scalar>::~LevenbergMarquardtForSerialLink() {
		}

		template <typename Scalar>
		const std::string LevenbergMarquardtForSerialLink<Scalar>::get_key() {
			return "LevenbergMarquardtForSerialLink";
		}

		template <typename Scalar>
		typename LevenbergMarquardtForSerialLink<Scalar>::Ptr LevenbergMarquardtForSerialLink<Scalar>::make_ptr(ModelPtr &model_ptr) {
			return std::make_unique<LevenbergMarquardtForSerialLink>(model_ptr);
		}

		template <>
		bool LevenbergMarquardtForSerialLink<double>::compute() {
			using Scalar = double;

			static const auto zero(Quantity::SpatialPoint<Scalar>::Point::Zero());
			static const auto shift_z_ray(Quantity::SpatialPoint<Scalar>::Point::UnitZ());
			static const auto shift_y_ray(Quantity::SpatialPoint<Scalar>::Point::UnitY());
			static const auto shift_x_ray(Quantity::SpatialPoint<Scalar>::Point::UnitX());

			std::vector<ControlPointMap<Scalar>::BodyID> body_ids;
			std::vector<RigidBodyDynamics::Math::Vector3d> body_points;
			std::vector<RigidBodyDynamics::Math::Vector3d> target_points;

			for(auto &[body_id, spatial_point] : this->control_point_map->access_to_this_storage()) {
				const auto inverse_quaternion = spatial_point.quaternion().inverse();
				const auto shift = spatial_point.point();

				body_ids.push_back(body_id);
				body_ids.push_back(body_id);
				body_ids.push_back(body_id);
				body_ids.push_back(body_id);

				body_points.push_back(zero);
				body_points.push_back(shift_z_ray);
				body_points.push_back(shift_y_ray);
				body_points.push_back(shift_x_ray);

				target_points.push_back(shift);
				target_points.push_back(shift + inverse_quaternion * shift_z_ray);
				target_points.push_back(shift + inverse_quaternion * shift_y_ray);
				target_points.push_back(shift + inverse_quaternion * shift_x_ray);
			}

			auto result_q = this->parameters->joint_angle()();

			auto is_success = RigidBodyDynamics::InverseKinematics(
				this->model->access(),
				this->parameters->joint_angle()(),
				body_ids,
				body_points,
				target_points,
				result_q,
				5e-4,
				1e-3,
				300
			);

			if(is_success) {
				this->parameters->joint_angle()() = result_q;
			}

			return is_success;
		}

		template <>
		[[deprecated("Please use double template.")]]
		bool LevenbergMarquardtForSerialLink<float>::compute() {
			using Scalar = double;

			static const auto zero(Quantity::SpatialPoint<Scalar>::Point::Zero());
			static const auto shift_z_ray(Quantity::SpatialPoint<Scalar>::Point::UnitZ());
			static const auto shift_y_ray(Quantity::SpatialPoint<Scalar>::Point::UnitY());
			static const auto shift_x_ray(Quantity::SpatialPoint<Scalar>::Point::UnitX());

			std::vector<ControlPointMap<Scalar>::BodyID> body_ids;
			std::vector<RigidBodyDynamics::Math::Vector3d> body_points;
			std::vector<RigidBodyDynamics::Math::Vector3d> target_points;

			for(auto &[body_id, spatial_point] : this->control_point_map->access_to_this_storage()) {
				body_ids.push_back(body_id);
				target_points.push_back(spatial_point.point().cast<Scalar>());
				body_points.push_back(zero);

				body_ids.push_back(body_id);
				target_points.push_back(spatial_point.point().cast<Scalar>() + spatial_point.quaternion().inverse().cast<Scalar>() * shift_z_ray);
				body_points.push_back(shift_z_ray);

				body_ids.push_back(body_id);
				target_points.push_back(spatial_point.point().cast<Scalar>() + spatial_point.quaternion().inverse().cast<Scalar>() * shift_y_ray);
				body_points.push_back(shift_y_ray);

				body_ids.push_back(body_id);
				target_points.push_back(spatial_point.point().cast<Scalar>() + spatial_point.quaternion().inverse().cast<Scalar>() * shift_x_ray);
				body_points.push_back(shift_x_ray);
			}

			RigidBodyDynamics::Math::VectorNd result_q = this->parameters->joint_angle()().cast<Scalar>();

			auto is_success = RigidBodyDynamics::InverseKinematics(
				this->model->access(),
				this->parameters->joint_angle()().cast<Scalar>(),
				body_ids,
				body_points,
				target_points,
				result_q,
				5e-4,
				1e-3,
				300
			);

			if(is_success) {
				this->parameters->joint_angle()() = result_q.cast<float>();
			}

			return is_success;
		}

		template <>
		bool LevenbergMarquardtForSerialLink<double>::compute(const Quantity::JointAngle<double> &init_q) {
			using Scalar = double;

			static const auto zero(Quantity::SpatialPoint<Scalar>::Point::Zero());
			static const auto shift_z_ray(Quantity::SpatialPoint<Scalar>::Point::UnitZ());
			static const auto shift_y_ray(Quantity::SpatialPoint<Scalar>::Point::UnitY());
			static const auto shift_x_ray(Quantity::SpatialPoint<Scalar>::Point::UnitX());

			std::vector<ControlPointMap<Scalar>::BodyID> body_ids;
			std::vector<RigidBodyDynamics::Math::Vector3d> body_points;
			std::vector<RigidBodyDynamics::Math::Vector3d> target_points;

			for(auto &[body_id, spatial_point] : this->control_point_map->access_to_this_storage()) {
				const auto inverse_quaternion = spatial_point.quaternion().inverse();
				const auto shift = spatial_point.point();

				body_ids.push_back(body_id);
				body_ids.push_back(body_id);
				body_ids.push_back(body_id);
				body_ids.push_back(body_id);

				body_points.push_back(zero);
				body_points.push_back(shift_z_ray);
				body_points.push_back(shift_y_ray);
				body_points.push_back(shift_x_ray);

				target_points.push_back(shift);
				target_points.push_back(shift + inverse_quaternion * shift_z_ray);
				target_points.push_back(shift + inverse_quaternion * shift_y_ray);
				target_points.push_back(shift + inverse_quaternion * shift_x_ray);
			}

			auto result_q = this->parameters->joint_angle()();

			auto is_success = RigidBodyDynamics::InverseKinematics(
				this->model->access(),
				init_q(),
				body_ids,
				body_points,
				target_points,
				result_q,
				1e-5,
				1e-2,
				1000
			);

			if(is_success) {
				this->parameters->joint_angle()() = result_q;
			}

			return is_success;
		}

		template <>
		[[deprecated("Please use double template.")]]
		bool LevenbergMarquardtForSerialLink<float>::compute(const Quantity::JointAngle<float> &init_q) {
			using Scalar = double;

			static const auto zero(Quantity::SpatialPoint<Scalar>::Point::Zero());
			static const auto shift_z_ray(Quantity::SpatialPoint<Scalar>::Point::UnitZ());
			static const auto shift_y_ray(Quantity::SpatialPoint<Scalar>::Point::UnitY());
			static const auto shift_x_ray(Quantity::SpatialPoint<Scalar>::Point::UnitX());

			std::vector<ControlPointMap<Scalar>::BodyID> body_ids;
			std::vector<RigidBodyDynamics::Math::Vector3d> body_points;
			std::vector<RigidBodyDynamics::Math::Vector3d> target_points;

			for(auto &[body_id, spatial_point] : this->control_point_map->access_to_this_storage()) {
				body_ids.push_back(body_id);
				target_points.push_back(spatial_point.point().cast<Scalar>());
				body_points.push_back(zero);

				body_ids.push_back(body_id);
				target_points.push_back(spatial_point.point().cast<Scalar>() + spatial_point.quaternion().inverse().cast<Scalar>() * shift_z_ray);
				body_points.push_back(shift_z_ray);

				body_ids.push_back(body_id);
				target_points.push_back(spatial_point.point().cast<Scalar>() + spatial_point.quaternion().inverse().cast<Scalar>() * shift_y_ray);
				body_points.push_back(shift_y_ray);

				body_ids.push_back(body_id);
				target_points.push_back(spatial_point.point().cast<Scalar>() + spatial_point.quaternion().inverse().cast<Scalar>() * shift_x_ray);
				body_points.push_back(shift_x_ray);
			}

			RigidBodyDynamics::Math::VectorNd result_q = this->parameters->joint_angle()().cast<Scalar>();

			auto is_success = RigidBodyDynamics::InverseKinematics(
				this->model->access(),
				init_q().cast<Scalar>(),
				body_ids,
				body_points,
				target_points,
				result_q,
				1e-5,
				1e-2,
				1000
			);

			if(is_success) {
				this->parameters->joint_angle()() = result_q.cast<float>();
			}

			return is_success;
		}

		template class LevenbergMarquardtForSerialLink<float>;
		template class LevenbergMarquardtForSerialLink<double>;
	}
}

