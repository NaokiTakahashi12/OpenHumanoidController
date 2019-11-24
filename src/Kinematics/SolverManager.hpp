
/**
  *
  * @file SolverManager.hpp
  * @author Naoki Takahashi
  *
  **/

#pragma once 

#include "Model/RBDLBased.hpp"
#include "Parameters.hpp"
#include "ControlPointMap.hpp"
#include "ForwardProblemSolvers/MultipleFKSelector.hpp"
#include "InverseProblemSolvers/MultipleIKSelector.hpp"

namespace Kinematics {
	template <typename Scalar>
	class SolverManager {
		public :
			using Ptr = std::unique_ptr<SolverManager>;

			using ModelPtr = Model::RBDLBased::Ptr;
			using FKSelector = ForwardProblemSolvers::MultipleFKSelector<Scalar>;
			using IKSelector = InverseProblemSolvers::MultipleIKSelector<Scalar>;
			using SolverNames = std::tuple<std::string, std::string>;
			using BeforeIKJointAngle = std::unique_ptr<Quantity::JointAngle<Scalar>>;

			SolverManager(ModelPtr &, const std::string &config_dir, const std::string &config_filename);
			virtual ~SolverManager();

			static Ptr make_ptr(ModelPtr &, const std::string &config_dir, const std::string &config_filename);

			SolverManager &entry_new_fk_solver(typename FKSelector::EntryObject);
			SolverManager &entry_new_ik_solver(typename IKSelector::EntryObject);

			SolverManager &entry_parameters(typename Parameters<Scalar>::Ptr &);
			SolverManager &entry_control_point_map(typename ControlPointMap<Scalar>::Ptr &);

			void spawn_solver();

			bool is_ready_fk();
			bool is_ready_ik();
			bool is_ready();

			bool first_ik();
			bool fk();
			bool ik();

		private :
			using FKSolver = ForwardProblemSolvers::MultipleFK<Scalar>;
			using IKSolver = InverseProblemSolvers::MultipleIK<Scalar>;

			const std::string config_dir,
				  			  config_filename;

			typename Parameters<Scalar>::Ptr parameters;
			typename ControlPointMap<Scalar>::Ptr control_point_map;

			typename FKSelector::Ptr fk_selector;
			typename IKSelector::Ptr ik_selector;

			typename FKSolver::Ptr fk_solver;
			typename IKSolver::Ptr ik_solver;

			BeforeIKJointAngle before_ik_joint_angle;

			void select_solver_from_config(const std::string &dir, const std::string &filename);
			void select_solver_from_names(const SolverNames);

			void entry_ik_joint_angle_from_config(const std::string &dir, const std::string &filename);
			void entry_ik_joint_angle(BeforeIKJointAngle);

			void fk_solver_selection(const std::string &);
			void ik_solver_selection(const std::string &);
	};
}

