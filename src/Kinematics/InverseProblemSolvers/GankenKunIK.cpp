
/**
  *
  * @file GankenKunIK.hpp
  * @author Yasuo Hayashibara
  *
  **/

#include "GankenKunIK.hpp"

#include <vector>
#include <iostream>

#include <rbdl/Kinematics.h>

using namespace GankenKun2018;

namespace Kinematics {
	namespace InverseProblemSolvers {
		template <typename Scalar>
		GankenKunIK<Scalar>::GankenKunIK(ModelPtr &new_model) : MultipleIK<Scalar>(new_model) {
		}

		template <typename Scalar>
		GankenKunIK<Scalar>::~GankenKunIK() {
		}

		template <typename Scalar>
		const std::string GankenKunIK<Scalar>::get_key() {
			return "GankenKunIK";
		}

		template <typename Scalar>
		typename GankenKunIK<Scalar>::Ptr GankenKunIK<Scalar>::make_ptr(ModelPtr &new_model) {
			return std::make_unique<GankenKunIK>(new_model);
		}

		template <>
		bool GankenKunIK<double>::compute() {
			if (is_first_compute) {
				link = new Link[Const::LINK_NUM];
				kine = new GankenKun2018::Kinematics(link);
				initLink(link);
				for(int i = 0; i < Const::SERVO_NUM; i ++) servo_angle[i] = 0.0;
				is_first_compute = false;
			}
			Link RFLink, LFLink;
			using Scalar = double;
			std::vector<ControlPointMap<Scalar>::BodyID> body_ids;
			for(auto &[body_id, spatial_point] : this->control_point_map->access_to_this_storage()) {
				if (body_id == Const::ANKLE_ROLL_R + 1) {
					auto pos = spatial_point.point();
					RFLink.p << pos[0], pos[1], pos[2];
				}
				if (body_id == Const::ANKLE_ROLL_L + 1) {
					auto pos = spatial_point.point();
					LFLink.p << pos[0], pos[1], pos[2];
				}
			}

			kine->calcInverseKinematics(servo_angle, RFLink, LFLink);
			for(int i = 0; i < Const::SERVO_NUM; i ++) {
				parameters->joint_angle()()(i) = servo_angle[i];
			}
#if 0
			printf("\r\ncompute inverse kinematics\r\n");
			std::cout << RFLink.p << std::endl;
			std::cout << LFLink.p << std::endl;
			printf("\r\n");
			for(int i = 0; i < Const::SERVO_NUM; i ++) {
				printf("%f ", servo_angle[i]);
			}
			printf("\r\n");
#endif
			return true;
		}

		template <>
		[[deprecated("Please use double template.")]]
		bool GankenKunIK<float>::compute() {
			if (is_first_compute) {
				link = new Link[Const::LINK_NUM];
				kine = new GankenKun2018::Kinematics(link);
				initLink(link);
				for(int i = 0; i < Const::SERVO_NUM; i ++) servo_angle[i] = 0.0;
				is_first_compute = false;
			}
			Link RFLink, LFLink;
			using Scalar = double;
			std::vector<ControlPointMap<Scalar>::BodyID> body_ids;
			for(auto &[body_id, spatial_point] : this->control_point_map->access_to_this_storage()) {
				if (body_id == Const::ANKLE_ROLL_R + 1) {
					auto pos = spatial_point.point();
					RFLink.p << pos[0], pos[1], pos[2];
				}
				if (body_id == Const::ANKLE_ROLL_L + 1) {
					auto pos = spatial_point.point();
					LFLink.p << pos[0], pos[1], pos[2];
				}
			}

			kine->calcInverseKinematics(servo_angle, RFLink, LFLink);
			for(int i = 0; i < Const::SERVO_NUM; i ++) {
				parameters->joint_angle()()(i) = servo_angle[i];
			}
			return true;
		}
		
		template <>
		bool GankenKunIK<double>::compute(const Quantity::JointAngle<double> &init_q) {
			if (is_first_compute) {
				link = new Link[Const::LINK_NUM];
				kine = new GankenKun2018::Kinematics(link);
				initLink(link);
				for(int i = 0; i < Const::SERVO_NUM; i ++) servo_angle[i] = 0.0;
				is_first_compute = false;
			}
			Link RFLink, LFLink;
			using Scalar = double;
			std::vector<ControlPointMap<Scalar>::BodyID> body_ids;
			for(auto &[body_id, spatial_point] : this->control_point_map->access_to_this_storage()) {
				if (body_id == Const::ANKLE_ROLL_R + 1) {
					auto pos = spatial_point.point();
					RFLink.p << pos[0], pos[1], pos[2];
				}
				if (body_id == Const::ANKLE_ROLL_L + 1) {
					auto pos = spatial_point.point();
					LFLink.p << pos[0], pos[1], pos[2];
				}
			}

			kine->calcInverseKinematics(servo_angle, RFLink, LFLink);
			for(int i = 0; i < Const::SERVO_NUM; i ++) {
				parameters->joint_angle()()(i) = servo_angle[i];
			}
			return true;
		}

		template <>
		[[deprecated("Please use double template.")]]
		bool GankenKunIK<float>::compute(const Quantity::JointAngle<float> &init_q) {
			if (is_first_compute) {
				link = new Link[Const::LINK_NUM];
				kine = new GankenKun2018::Kinematics(link);
				initLink(link);
				for(int i = 0; i < Const::SERVO_NUM; i ++) servo_angle[i] = 0.0;
				is_first_compute = false;
			}
			Link RFLink, LFLink;
			using Scalar = double;
			std::vector<ControlPointMap<Scalar>::BodyID> body_ids;
			for(auto &[body_id, spatial_point] : this->control_point_map->access_to_this_storage()) {
				if (body_id == Const::ANKLE_ROLL_R + 1) {
					auto pos = spatial_point.point();
					RFLink.p << pos[0], pos[1], pos[2];
				}
				if (body_id == Const::ANKLE_ROLL_L + 1) {
					auto pos = spatial_point.point();
					LFLink.p << pos[0], pos[1], pos[2];
				}
			}

			kine->calcInverseKinematics(servo_angle, RFLink, LFLink);
			for(int i = 0; i < Const::SERVO_NUM; i ++) {
				parameters->joint_angle()()(i) = servo_angle[i];
			}
			return true;
		}

		template class GankenKunIK<float>;
		template class GankenKunIK<double>;
	}
}

