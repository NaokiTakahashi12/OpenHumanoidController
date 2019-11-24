
#pragma once

#include <map>
#include <functional>

#include "Parameters.hpp"

namespace Kinematics {
	template <typename Scalar>
	class JointAngleModificator {
		public :
			using Ptr = std::unique_ptr<JointAngleModificator>;
			using OverrideRoundFunction = std::function<bool(Quantity::JointAngle<Scalar> &)>;

			JointAngleModificator(typename Parameters<Scalar>::Ptr &, const std::string &dir, const std::string &config_file);
			virtual ~JointAngleModificator();

			static Ptr make_ptr(typename Parameters<Scalar>::Ptr &, const std::string &dir, const std::string &config_file);

			void override_round_function(OverrideRoundFunction);

			void assert_round();
			bool round();

			void assert_limit_angle();
			bool is_limit_angle();

			void assert_modify();
			bool modify();

		private :
			using LimitPair = std::pair<float, float>;
			using LimitMap = std::map<int, LimitPair>;

			typename Parameters<Scalar>::Ptr parameters;

			OverrideRoundFunction round_function;

			LimitMap limit_map;

	};
}

