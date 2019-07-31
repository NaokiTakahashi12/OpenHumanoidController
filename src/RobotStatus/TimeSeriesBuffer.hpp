
/**
  *
  * @file TimeSeriesBuffer.hpp
  * @brief RobotStatus data buffer class
  * @author NaokiTakahashi
  *
  **/

#pragma once

#include <cstddef>
#include <memory>
#include <vector>
#include <deque>

#include <Tools/Math/Matrix.hpp>

#include "TimeSeriesData.hpp"
#include "Buffer.hpp"

namespace RobotStatus {
	template <typename T>
	class TimeSeriesBuffer : Buffer<TimeSeriesData<T>> {
		public:
			TimeSeriesBuffer(const typename Buffer<TimeSeriesData<T>>::SizeType &size);
	};

	template <typename T>
	using TimeSeriesVectorX = TimeSeriesBuffer<Tools::Math::VectorX<T>>;

	template <typename T>
	using TimeSeriesMatrixX = TimeSeriesBuffer<Tools::Math::MatrixX<T>>;

	using TimeSeriesBufferi = TimeSeriesBuffer<int>;
	using TimeSeriesBufferf = TimeSeriesBuffer<float>;
	using TimeSeriesBufferd = TimeSeriesBuffer<double>;

	using TimeSeriesVectorXi = TimeSeriesVectorX<int>;
	using TimeSeriesVectorXf = TimeSeriesVectorX<float>;
	using TimeSeriesVectorXd = TimeSeriesVectorX<double>;

	using TimeSeriesMatrixXi = TimeSeriesMatrixX<int>;
	using TimeSeriesMatrixXf = TimeSeriesMatrixX<float>;
	using TimeSeriesMatrixXd = TimeSeriesMatrixX<double>;
}

