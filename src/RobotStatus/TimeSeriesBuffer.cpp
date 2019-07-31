
/**
  *
  * @file TimeSeriesBuffer.cpp
  * @author NaokiTakahashi
  *
  **/

#include "TimeSeriesBuffer.hpp"

namespace RobotStatus {
	template <typename T>
	TimeSeriesBuffer<T>::TimeSeriesBuffer(const typename Buffer<TimeSeriesData<T>>::SizeType &size) : Buffer<TimeSeriesData<T>>(size) {
	}

	/*
	template class TimeSeriesBuffer<int>;
	template class TimeSeriesBuffer<float>;
	template class TimeSeriesBuffer<double>;

	template class TimeSeriesBuffer<Tools::Math::MatrixX<int>>;
	template class TimeSeriesBuffer<Tools::Math::MatrixX<float>>;
	template class TimeSeriesBuffer<Tools::Math::MatrixX<double>>;

	template class TimeSeriesBuffer<Tools::Math::Matrix2<int>>;
	template class TimeSeriesBuffer<Tools::Math::Matrix2<float>>;
	template class TimeSeriesBuffer<Tools::Math::Matrix2<double>>;

	template class TimeSeriesBuffer<Tools::Math::Matrix3<int>>;
	template class TimeSeriesBuffer<Tools::Math::Matrix3<float>>;
	template class TimeSeriesBuffer<Tools::Math::Matrix3<double>>;

	template class TimeSeriesBuffer<Tools::Math::Matrix4<int>>;
	template class TimeSeriesBuffer<Tools::Math::Matrix4<float>>;
	template class TimeSeriesBuffer<Tools::Math::Matrix4<double>>;

	template class TimeSeriesBuffer<Tools::Math::VectorX<int>>;
	template class TimeSeriesBuffer<Tools::Math::VectorX<float>>;
	template class TimeSeriesBuffer<Tools::Math::VectorX<double>>;

	template class TimeSeriesBuffer<Tools::Math::Vector2<int>>;
	template class TimeSeriesBuffer<Tools::Math::Vector2<float>>;
	template class TimeSeriesBuffer<Tools::Math::Vector2<double>>;

	template class TimeSeriesBuffer<Tools::Math::Vector3<int>>;
	template class TimeSeriesBuffer<Tools::Math::Vector3<float>>;
	template class TimeSeriesBuffer<Tools::Math::Vector3<double>>;

	template class TimeSeriesBuffer<Tools::Math::Vector4<int>>;
	template class TimeSeriesBuffer<Tools::Math::Vector4<float>>;
	template class TimeSeriesBuffer<Tools::Math::Vector4<double>>;
	*/
}

