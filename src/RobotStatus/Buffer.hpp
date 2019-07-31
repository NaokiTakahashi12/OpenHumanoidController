
/**
  *
  * @file Buffer.hpp
  * @brief RobotStatus data buffer class
  * @author NaokiTakahashi
  *
  **/

#pragma once

#include <cstddef>
#include <memory>
#include <vector>
#include <deque>

namespace RobotStatus {
	template <typename T>
	class Buffer {
		public :
			using Data = std::vector<T>;
			using Iterator = typename Data::iterator;
			using SizeType = typename Data::size_type;

			Buffer(const SizeType &maximum_data_size);

			void resize(const SizeType &maximum_data_size);
			void reset();

			void push(const T &);

			SizeType size();
			T &at(const SizeType &); 
			Data get_raw_data();
			Data get_sorted_datas();

		protected :
			SizeType maximum_data_size;

			std::unique_ptr<Data> data_list;
			std::unique_ptr<Iterator> data_iterator;
	};
}

