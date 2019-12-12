
/**
  *
  * @file ControlPointMap.cpp
  * @author Naoki Takahashi
  *
  **/

#include "ControlPointMap.hpp"

#include <stdexcept>

namespace Kinematics {
	template <typename Scalar>
	ControlPointMap<Scalar>::ControlPointMap() {
		access_map_mutex = std::make_unique<std::mutex>();
	}

	template <typename Scalar>
	ControlPointMap<Scalar>::~ControlPointMap() {
	}

	template <typename Scalar>
	typename ControlPointMap<Scalar>::Ptr ControlPointMap<Scalar>::make_ptr() {
		return std::make_shared<ControlPointMap>();
	}

	template <typename Scalar>
	void ControlPointMap<Scalar>::clear() {
		const auto lock = std::lock_guard<std::mutex>(*access_map_mutex);

		storage.clear();
	}

	template <typename Scalar>
	void ControlPointMap<Scalar>::set(const BodyID &body_id) {
		const auto lock = std::lock_guard<std::mutex>(*access_map_mutex);

		if(is_registered(body_id)) {
			throw std::runtime_error("Already registered body id: " + std::to_string(body_id) + "from Kinematics::ControlPointMap");
		}

		storage[body_id] = Quantity::SpatialPoint<Scalar>();
	}

	template <typename Scalar>
	void ControlPointMap<Scalar>::set(const BodyID &body_id, const Quantity::SpatialPoint<Scalar> &spatial_point) {
		const auto lock = std::lock_guard<std::mutex>(*access_map_mutex);

		if(is_registered(body_id)) {
			throw std::runtime_error("Already registered body id: " + std::to_string(body_id) + " from Kinematics::ControlPointMap");
		}

		storage[body_id] = spatial_point;
	}

	template <typename Scalar>
	void ControlPointMap<Scalar>::update(const BodyID &body_id, const Quantity::SpatialPoint<Scalar> &spatial_point) {
		const auto lock = std::lock_guard<std::mutex>(*access_map_mutex);

		if(!is_registered(body_id)) {
			throw std::runtime_error("Can not registered body id: " + std::to_string(body_id) + " from Kinematics::ControlPointMap");
		}

		storage[body_id] = spatial_point;
	}

	template <typename Scalar>
	void ControlPointMap<Scalar>::add(const BodyID &body_id, const Quantity::SpatialPoint<Scalar> &spatial_point) {
		const auto lock = std::lock_guard<std::mutex>(*access_map_mutex);

		if(!is_registered(body_id)) {
			throw std::runtime_error("Can not registered body id: " + std::to_string(body_id) + " from Kinematics::ControlPointMap");
		}

		storage[body_id] += spatial_point;
	}

	template <typename Scalar>
	bool ControlPointMap<Scalar>::is_registered(const BodyID &body_id) {
		return storage.find(body_id) != storage.cend();
	}

	template <typename Scalar>
	Quantity::SpatialPoint<Scalar> ControlPointMap<Scalar>::get_point(const BodyID &body_id) {
		const auto lock = std::lock_guard<std::mutex>(*access_map_mutex);

		return storage[body_id];
	}

	template <typename Scalar>
	typename ControlPointMap<Scalar>::DataList ControlPointMap<Scalar>::get_list() {
		DataList ret_list;

		for(auto &once : storage) {
			ret_list.push_back(once.second);
		}

		return ret_list;
	}

	template <typename Scalar>
	typename ControlPointMap<Scalar>::DataTupleList ControlPointMap<Scalar>::get_list_with_id() {
		DataTupleList ret_list;

		for(auto &[id, data] : storage) {
			ret_list.push_back({id, data});
		}

		return ret_list;
	}

	template <typename Scalar>
	typename ControlPointMap<Scalar>::Storage &ControlPointMap<Scalar>::access_to_this_storage() {
		return storage;
	}

	template <typename Scalar>
	ControlPointMap<Scalar> &ControlPointMap<Scalar>::operator = (const ControlPointMap &control_point_map) {
		if(this != &control_point_map) {
			this->storage = control_point_map.storage;
		}
		return *this;
	}

	template class ControlPointMap<float>;
	template class ControlPointMap<double>;
	template class ControlPointMap<long double>;
}

