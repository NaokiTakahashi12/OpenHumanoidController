
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

	template struct TimeSeriesData<uint8_t>;
	template struct TimeSeriesData<uint32_t>;
	template struct TimeSeriesData<int>;
	template struct TimeSeriesData<float>;
	template struct TimeSeriesData<double>;

	template struct TimeSeriesData<Tools::Math::VectorX<int>>;
	template struct TimeSeriesData<Tools::Math::VectorX<float>>;
	template struct TimeSeriesData<Tools::Math::VectorX<double>>;
	template struct TimeSeriesData<Tools::Math::Vector2<float>>;
	template struct TimeSeriesData<Tools::Math::Vector3<float>>;
	template struct TimeSeriesData<Tools::Math::Vector4<float>>;
	template struct TimeSeriesData<Tools::Math::Vector2<double>>;
	template struct TimeSeriesData<Tools::Math::Vector3<double>>;
	template struct TimeSeriesData<Tools::Math::Vector4<double>>;
	template struct TimeSeriesData<Tools::Math::MatrixX<int>>;
	template struct TimeSeriesData<Tools::Math::MatrixX<float>>;
	template struct TimeSeriesData<Tools::Math::MatrixX<double>>;
	template struct TimeSeriesData<Tools::Math::Matrix2<float>>;
	template struct TimeSeriesData<Tools::Math::Matrix3<float>>;
	template struct TimeSeriesData<Tools::Math::Matrix4<float>>;
	template struct TimeSeriesData<Tools::Math::Matrix2<double>>;
	template struct TimeSeriesData<Tools::Math::Matrix3<double>>;
	template struct TimeSeriesData<Tools::Math::Matrix4<double>>;
}

