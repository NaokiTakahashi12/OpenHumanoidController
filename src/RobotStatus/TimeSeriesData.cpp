
/**
  *
  * @file TimeSeriesData.cpp
  * @auther Naoki Takahashi
  *
  **/

#include "TimeSeriesData.hpp"

#include <Tools/Math/Matrix.hpp>

namespace RobotStatus {
	template <typename T>
	TimeSeriesData<T>::TimeSeriesData() {
	}

	template <typename T>
	TimeSeriesData<T>::TimeSeriesData(const TimeSeriesData<T> &time_series_data) {
		if(this != &time_series_data) {
			this->value = time_series_data.value;
			this->timestamp = time_series_data.timestamp;
		}
	}

	template <typename T>
	TimeSeriesData<T> &TimeSeriesData<T>::operator = (const TimeSeriesData &data) {
		if(this != &data) {
			this->value = data.value;
			this->timestamp = data.timestamp;
		}
		return *this;
	}

	template <typename T>
	bool TimeSeriesData<T>::operator == (const TimeSeriesData &data) const {
		return this->timestamp == data.timestamp;
	}

	template <typename T>
	bool TimeSeriesData<T>::operator != (const TimeSeriesData &data) const {
		return this->timestamp != data.timestamp;
	}

	template <typename T>
	bool TimeSeriesData<T>::operator <= (const TimeSeriesData &data) const {
		return this->timestamp <= data.timestamp;
	}

	template <typename T>
	bool TimeSeriesData<T>::operator >= (const TimeSeriesData &data) const {
		return this->timestamp >= data.timestamp;
	}

	template <typename T>
	bool TimeSeriesData<T>::operator < (const TimeSeriesData &data) const {
		return this->timestamp < data.timestamp;
	}

	template <typename T>
	bool TimeSeriesData<T>::operator > (const TimeSeriesData &data) const {
		return this->timestamp > data.timestamp;
	}

	template class TimeSeriesData<int>;
	template class TimeSeriesData<float>;
	template class TimeSeriesData<double>;

	template class TimeSeriesData<Tools::Math::VectorX<int>>;
	template class TimeSeriesData<Tools::Math::VectorX<float>>;
	template class TimeSeriesData<Tools::Math::VectorX<double>>;

	template class TimeSeriesData<Tools::Math::Vector2<int>>;
	template class TimeSeriesData<Tools::Math::Vector2<float>>;
	template class TimeSeriesData<Tools::Math::Vector2<double>>;

	template class TimeSeriesData<Tools::Math::Vector3<int>>;
	template class TimeSeriesData<Tools::Math::Vector3<float>>;
	template class TimeSeriesData<Tools::Math::Vector3<double>>;

	template class TimeSeriesData<Tools::Math::Vector4<int>>;
	template class TimeSeriesData<Tools::Math::Vector4<float>>;
	template class TimeSeriesData<Tools::Math::Vector4<double>>;

	template class TimeSeriesData<Tools::Math::MatrixX<int>>;
	template class TimeSeriesData<Tools::Math::MatrixX<float>>;
	template class TimeSeriesData<Tools::Math::MatrixX<double>>;

	template class TimeSeriesData<Tools::Math::Matrix2<int>>;
	template class TimeSeriesData<Tools::Math::Matrix2<float>>;
	template class TimeSeriesData<Tools::Math::Matrix2<double>>;

	template class TimeSeriesData<Tools::Math::Matrix3<int>>;
	template class TimeSeriesData<Tools::Math::Matrix3<float>>;
	template class TimeSeriesData<Tools::Math::Matrix3<double>>;

	template class TimeSeriesData<Tools::Math::Matrix4<int>>;
	template class TimeSeriesData<Tools::Math::Matrix4<float>>;
	template class TimeSeriesData<Tools::Math::Matrix4<double>>;
}

