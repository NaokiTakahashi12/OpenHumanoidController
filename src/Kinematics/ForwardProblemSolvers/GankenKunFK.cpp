
/**
  *
  * @file GankenKunFK.hpp
  * @author Yasuo Hayashibara
  *
  **/

#include "GankenKunFK.hpp"

using namespace GankenKun2018;

namespace Kinematics {
	namespace ForwardProblemSolvers {
		template <typename Scalar>
		GankenKunFK<Scalar>::GankenKunFK(ModelPtr &new_model) : MultipleFK<Scalar>(new_model) {
			link = new Link[Const::LINK_NUM];
			kine = new GankenKun2018::Kinematics(link);
			initLink(link);
		}

		template <typename Scalar>
		GankenKunFK<Scalar>::~GankenKunFK() {
			delete(kine);
			delete(link);
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
			for(int i = 0; i < Const::SERVO_NUM; i ++)
				servo_angle[i] = parameters->joint_angle()()(i);

			kine->setJointAngle(servo_angle);
			kine->calcForwardKinematics();

			for(auto &&[body_id, spatial_point] : this->control_point_map->access_to_this_storage()) {
				if (body_id == Const::ANKLE_ROLL_R + 1) {
					auto p = link[Const::RR2].p;
					spatial_point.point(p[0], p[1], p[2]);
				}
				if (body_id == Const::ANKLE_ROLL_L + 1) {
					auto p = link[Const::LR2].p;
					spatial_point.point(p[0], p[1], p[2]);
				}
			}
			for(int i = 0; i < Const::SERVO_NUM; i ++) {
				parameters->joint_angle()()(i) = servo_angle[i];
			}
#if 0
			printf("\r\ncompute forward kinematics\r\n");
			for(int i = 0; i < Const::SERVO_NUM; i ++) {
				printf("%f ", servo_angle[i]);
			}
			std::cout << link[Const::RR2].p << std::endl;
			std::cout << link[Const::LR2].p << std::endl;
			printf("\r\n");
#endif
			return true;
		}

		template <>
		[[deprecated("Please use double template.")]]
		bool GankenKunFK<float>::compute() {
			return true;
		}

		template class GankenKunFK<float>;
		template class GankenKunFK<double>;
	}
}

