
/**
  *
  * @file Launcher.hpp
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include <atomic>

#include <RobotStatus/Information.hpp>
#include <Tools/Math/Matrix.hpp>

#include "LinerInvertedPendulum/WalkFragments.hpp"

namespace TrajectoryPattern {
	class Launcher {
		public :
			using Footprints = Tools::Math::MatrixX<LinerInvertedPendulum::WalkFragments::MatrixElement>;

			Launcher(RobotStatus::InformationPtr &);
			~Launcher();

			void set_config_file(const std::string &dir, const std::string &filename);

			bool is_computing_now() const;
			void wait_for_computing() const;

			void operator ()();

			std::unique_ptr<LinerInvertedPendulum::WalkFragments> liner_inverted_pendulum;

		private :
			std::string config_dir,
						config_filename;

			std::atomic<bool> computing_now;

			float delay_raising_foot;
			float raising_foot;
			float com_hight;
			float plane_contact_time;
			float minimize_a,
				  minimize_b;
						
			std::unique_ptr<std::thread> async_update_com_trajectory_thread;

			std::unique_ptr<Footprints> footprints;

			RobotStatus::InformationPtr robo_info;

			void initialize();
	};
}

