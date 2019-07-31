
/**
  *
  * @file Buffer.cpp
  * @author NaokiTakahashi
  *
  **/

#include "Buffer.hpp"

#include <stdexcept>
#include <algorithm>

#include <Tools/Math/Matrix.hpp>

#include "TimeSeriesData.hpp"

namespace RobotStatus {
	template <typename T>
	Buffer<T>::Buffer(const SizeType &maximum_data_size) {
		resize(maximum_data_size);
	}

	template <typename T>
	void Buffer<T>::resize(const SizeType &maximum_data_size) {
		if(0 >= maximum_data_size) {
			throw std::out_of_range("Data size too small from RobotStatus::Buffer::resize");
		}
		this->maximum_data_size = maximum_data_size;
		reset();
	}

	template <typename T>
	void Buffer<T>::reset() {
		if(data_list) {
			data_list->clear();
		}
		data_list = std::make_unique<Data>(maximum_data_size);
		data_iterator = std::unique_ptr<Iterator>();
		*data_iterator = data_list->begin();
	}

	template <typename T>
	void Buffer<T>::push(const T &data) {
		std::advance(*data_iterator, 1);
		if(*data_iterator >= data_list->end()) {
			*data_iterator = data_list->begin();
		}
		**data_iterator = data;
	}

	template <typename T>
	typename Buffer<T>::SizeType Buffer<T>::size() {
		return data_list->size();
	}

	template <typename T>
	T &Buffer<T>::at(const SizeType &n) {
		if(maximum_data_size < n || 0 > n) {
			throw std::out_of_range("Illegal access from RobotStatus::Buffer::at");
		}
		static SizeType current_n;
		current_n = std::distance(data_list->begin(), *data_iterator);
		if(n > current_n) {
			return data_list->at(current_n + maximum_data_size - n);
		}
		return data_list->at(current_n - n);
	}

	template <typename T>
	typename Buffer<T>::Data Buffer<T>::get_raw_data() {
		return *data_list;
	}

	template <typename T>
	typename Buffer<T>::Data Buffer<T>::get_sorted_datas() {
		auto return_data = *data_list;
		std::sort(return_data.begin(), return_data.end());
		return return_data;
	}

	template class Buffer<int>;
	template class Buffer<float>;
	template class Buffer<double>;

	template class Buffer<TimeSeriesData<int>>;
	template class Buffer<TimeSeriesData<float>>;
	template class Buffer<TimeSeriesData<double>>;

	template class Buffer<TimeSeriesData<Tools::Math::VectorX<int>>>;
	template class Buffer<TimeSeriesData<Tools::Math::VectorX<float>>>;
	template class Buffer<TimeSeriesData<Tools::Math::VectorX<double>>>;

	template class Buffer<TimeSeriesData<Tools::Math::Vector2<int>>>;
	template class Buffer<TimeSeriesData<Tools::Math::Vector2<float>>>;
	template class Buffer<TimeSeriesData<Tools::Math::Vector2<double>>>;

	template class Buffer<TimeSeriesData<Tools::Math::Vector3<int>>>;
	template class Buffer<TimeSeriesData<Tools::Math::Vector3<float>>>;
	template class Buffer<TimeSeriesData<Tools::Math::Vector3<double>>>;

	template class Buffer<TimeSeriesData<Tools::Math::Vector4<int>>>;
	template class Buffer<TimeSeriesData<Tools::Math::Vector4<float>>>;
	template class Buffer<TimeSeriesData<Tools::Math::Vector4<double>>>;

	template class Buffer<TimeSeriesData<Tools::Math::MatrixX<int>>>;
	template class Buffer<TimeSeriesData<Tools::Math::MatrixX<float>>>;
	template class Buffer<TimeSeriesData<Tools::Math::MatrixX<double>>>;

	template class Buffer<TimeSeriesData<Tools::Math::Matrix2<int>>>;
	template class Buffer<TimeSeriesData<Tools::Math::Matrix2<float>>>;
	template class Buffer<TimeSeriesData<Tools::Math::Matrix2<double>>>;

	template class Buffer<TimeSeriesData<Tools::Math::Matrix3<int>>>;
	template class Buffer<TimeSeriesData<Tools::Math::Matrix3<float>>>;
	template class Buffer<TimeSeriesData<Tools::Math::Matrix3<double>>>;

	template class Buffer<TimeSeriesData<Tools::Math::Matrix4<int>>>;
	template class Buffer<TimeSeriesData<Tools::Math::Matrix4<float>>>;
	template class Buffer<TimeSeriesData<Tools::Math::Matrix4<double>>>;
}

