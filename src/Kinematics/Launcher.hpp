
/**
  *
  * @file Launcher.hpp
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include <mutex>
#include <thread>

#include "Model/RBDLBased.hpp"
#include "Parameters.hpp"
#include "ControlPointMap.hpp"
#include "SolverManager.hpp"
#include "JointAngleModificator.hpp"

namespace Kinematics {
	template <typename Scalar>
	class Launcher {
		private :
			using FKSelector = typename SolverManager<Scalar>::FKSelector;
			using IKSelector = typename SolverManager<Scalar>::IKSelector;

		public :
			using FKObject = typename FKSelector::EntryObject;
			using IKObject = typename IKSelector::EntryObject;

			Launcher(const std::string &dir, const std::string &config_file);
			virtual ~Launcher();

			void initialize();

			void close();

			void entry_new_fk_solver(FKObject);
			void entry_new_ik_solver(IKObject);

			typename Parameters<Scalar>::Ptr get_parameters();
			typename ControlPointMap<Scalar>::Ptr get_control_point_map();
			Model::RBDLBased::Ptr get_model();

			std::mutex &get_mutex();

			void operator () ();

		private :
			using RuntimeThread = std::unique_ptr<std::thread>;

			bool enable_loop;

			const std::string directory,
				  			  config_file;

			RuntimeThread runtime_thread;

			Model::RBDLBased::Ptr model;

			std::mutex access_mutex;

			typename Parameters<Scalar>::Ptr parameters;
			typename ControlPointMap<Scalar>::Ptr control_point_map;

			typename SolverManager<Scalar>::Ptr solver_manager;
			typename JointAngleModificator<Scalar>::Ptr joint_angle_modificator;

			void make_model(const std::string &model_file);
			void set_control_point_from_config_for_humanoid(const std::string &config_file);

			bool is_update(typename ControlPointMap<Scalar>::DataList &);
	};
}

