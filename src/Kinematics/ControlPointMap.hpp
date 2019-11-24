
/**
  *
  * @file ControlPointMap.hpp
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include <unordered_map>
#include <mutex>
#include <tuple>
#include <vector>
#include <memory>

#include "Quantity/SpatialPoint.hpp"

namespace Kinematics {
	template <typename Scalar>
	class ControlPointMap {
		public :
			using Ptr = std::shared_ptr<ControlPointMap>;

			using BodyID = unsigned int;

			using DataTuple = std::tuple<BodyID, Quantity::SpatialPoint<Scalar>>;
			using DataTupleList = std::vector<DataTuple>;
			using DataList = std::vector<Quantity::SpatialPoint<Scalar>>;

		private :
			using Storage = std::unordered_map<BodyID, Quantity::SpatialPoint<Scalar>>;

		public :
			ControlPointMap();
			virtual  ~ControlPointMap();

			static Ptr make_ptr();

			void clear();

			void set(const BodyID &);
			void set(const BodyID &, const Quantity::SpatialPoint<Scalar> &);

			void update(const BodyID &, const Quantity::SpatialPoint<Scalar> &);

			void add(const BodyID &, const Quantity::SpatialPoint<Scalar> &);

			bool is_registered(const BodyID &);

			Quantity::SpatialPoint<Scalar> get_point(const BodyID &);

			DataList get_list();
			DataTupleList get_list_with_id();

			Storage &access_to_this_storage();

			ControlPointMap &operator = (const ControlPointMap &);

		private :
			std::unique_ptr<std::mutex> access_map_mutex;

			Storage storage;
	};
}

