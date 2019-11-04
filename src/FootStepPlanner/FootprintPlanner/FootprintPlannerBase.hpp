
/**
  *
  * @file FootprintPlannerBase.hpp @brief Footprint base class @author Naoki
  * Takahashi
  *
  **/

#pragma once 

#include "../Footprint/SpatialPoint.hpp"
#include "../ConfigManager.hpp"

namespace FootStepPlanner {
	namespace FootprintPlanner {
		template <typename Scalar = double>
		class FootprintPlannerBase {
			protected :
				static constexpr auto dimention_rank = 3;

				using SpatialPoint = Footprint::SpatialPoint<Scalar, dimention_rank>;
				using SpatialPointPtr = typename SpatialPoint::Ptr;

				SpatialPointPtr begin_point, goal_point;

				ConfigManager::Ptr config_manager;

			public :
				using Vector = typename SpatialPoint::Vector;
				using EulerAngles = typename SpatialPoint::EulerAngles;

				using Ptr = std::unique_ptr<FootprintPlannerBase>;

				FootprintPlannerBase();
				virtual ~FootprintPlannerBase();

				FootprintPlannerBase &set_goal(const decltype((goal_point)));
				FootprintPlannerBase &set_goal(const Vector &, const EulerAngles &);
				FootprintPlannerBase &set_goal(const Scalar &x, const Scalar &y, const Scalar &z, const Scalar &a, const Scalar &b, const Scalar &g);
				FootprintPlannerBase &set_begin(const decltype((begin_point)));
				FootprintPlannerBase &set_begin(const Vector &, const EulerAngles &);
				FootprintPlannerBase &set_begin(const Scalar &x, const Scalar &y, const Scalar &z, const Scalar &a, const Scalar &b, const Scalar &g);

				virtual void config(const std::string &new_config_file_name);

				virtual void clear_footprint();

		};
	}
}

