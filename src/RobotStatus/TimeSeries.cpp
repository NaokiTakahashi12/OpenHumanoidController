
/**
  *
  * @file TimeSeries.cpp
  * @auther Naoki Takahashi
  *
  **/

#include "TimeSeries.hpp"

#include <algorithm>
#include <exception>
#include <iostream>

#include <Tools/Math/Matrix.hpp>

namespace RobotStatus {
	template <typename T>
	TimeSeries<T>::TimeSeries(const int hold_data_size) {
		if(hold_data_size < 0) {
			throw std::runtime_error("Illegal size from RobotStatus::TimeSeries");
		}
		maximum_size_of_hold_list = hold_data_size;
		reset();
	}

	template <typename T>
	TimeSeries<T> &TimeSeries<T>::operator = (const TimeSeries<T> &time_series) {
		if(this != &time_series) {
			this->maximum_size_of_hold_list = time_series.maximum_size_of_hold_list;
			this->seek_iterator = time_series.seek_iterator;
			this->start_nanoseconds = std::make_unique<NanoSeconds>(*time_series.start_nanoseconds);
			this->hold_list = std::make_unique<HoldList>(*time_series.hold_list);
		}
		return *this;
	}

	template <typename T>
	typename TimeSeries<T>::HoldType TimeSeries<T>::latest() {
		return *seek_iterator;
	}

	template <typename T>
	typename TimeSeries<T>::HoldType TimeSeries<T>::at(const int &position) {
		static int number_of_current_position;

		number_of_current_position = std::distance(hold_list->begin(), seek_iterator);

		if(maximum_size_of_hold_list <= position || 0 > position) {
			throw std::runtime_error("Illegal access from RobotStatus::TimeSeries");
		}
		else if(position > number_of_current_position) {
			return hold_list->at(number_of_current_position + maximum_size_of_hold_list - position);
		}

		return hold_list->at(number_of_current_position - position);
	}

	template <typename T>
	typename TimeSeries<T>::HoldList TimeSeries<T>::get_raw() {
		return *hold_list;
	}

	template <typename T>
	typename TimeSeries<T>::HoldList TimeSeries<T>::get_all() {
		static auto return_list = *hold_list;
		std::sort(return_list.begin(), return_list.end());
		return return_list;
	}

	template <typename T>
	void TimeSeries<T>::set(const T &value) {
		++ seek_iterator;

		if(seek_iterator >= hold_list->cend()) {
			seek_iterator -= hold_list->size();
		}

		seek_iterator->value = value;
		seek_iterator->timestamp = (get_current_nanoseconds() - *start_nanoseconds).count();
	}

	template <typename T>
	void TimeSeries<T>::set(const T &value, const typename HoldType::TimestampType &timestamp) {
		++ seek_iterator;

		if(seek_iterator >= hold_list->cend()) {
			seek_iterator -= hold_list->size();
		}

		seek_iterator->value = value;
		seek_iterator->timestamp = timestamp;
	}

	template <typename T>
	void TimeSeries<T>::reset() {
		hold_list = std::make_unique<HoldList>(maximum_size_of_hold_list);
		if(hold_list->size() <= 0) {
			throw std::runtime_error("Failed create of hold list");
		}
		zero_reset();
		reset_timer();
	}

	template <typename T>
	void TimeSeries<T>::reset_timer() {
		start_nanoseconds = std::make_unique<NanoSeconds>(get_current_nanoseconds());
	}

	template <typename T>
	void TimeSeries<T>::zero_reset() {
		for(auto &&hl : *(hold_list)) {
			hl.value *= 0;
			hl.timestamp = 0;
		}
		seek_iterator = hold_list->begin();
	}

	template <typename T>
	int TimeSeries<T>::hold_size() const {
		return maximum_size_of_hold_list;
	}

	template <typename T>
	typename TimeSeries<T>::NanoSeconds TimeSeries<T>::get_current_nanoseconds() {
		return std::chrono::duration_cast<NanoSeconds>(TimeType::now().time_since_epoch());
	}

	template class TimeSeries<int>;
	template class TimeSeries<float>;
	template class TimeSeries<double>;

	template class TimeSeries<Tools::Math::VectorX<int>>;
	template class TimeSeries<Tools::Math::VectorX<float>>;
	template class TimeSeries<Tools::Math::VectorX<double>>;
	template class TimeSeries<Tools::Math::Vector2<float>>;
	template class TimeSeries<Tools::Math::Vector3<float>>;
	template class TimeSeries<Tools::Math::Vector4<float>>;
	template class TimeSeries<Tools::Math::Vector2<double>>;
	template class TimeSeries<Tools::Math::Vector3<double>>;
	template class TimeSeries<Tools::Math::Vector4<double>>;
	template class TimeSeries<Tools::Math::MatrixX<int>>;
	template class TimeSeries<Tools::Math::MatrixX<float>>;
	template class TimeSeries<Tools::Math::MatrixX<double>>;
	template class TimeSeries<Tools::Math::Matrix2<float>>;
	template class TimeSeries<Tools::Math::Matrix3<float>>;
	template class TimeSeries<Tools::Math::Matrix4<float>>;
	template class TimeSeries<Tools::Math::Matrix2<double>>;
	template class TimeSeries<Tools::Math::Matrix3<double>>;
	template class TimeSeries<Tools::Math::Matrix4<double>>;
}

