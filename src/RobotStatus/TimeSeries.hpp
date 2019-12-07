
/**
  *
  * @file TimeSeries.hpp
  * @auther Naoki Takahashi
  *
  **/

#pragma once

#include <memory>
#include <iterator>
#include <vector>
#include <mutex>
#include <chrono>

//#include "TimeSeriesBuffer.hpp"
#include "TimeSeriesData.hpp"

namespace RobotStatus {
	template <typename T>
	class TimeSeries final {
		public :
			using TimeType = std::chrono::high_resolution_clock;
			using NanoSeconds = std::chrono::nanoseconds;
			using HoldType = TimeSeriesData<T>;
			using HoldList = std::vector<HoldType>;
			using HoldListIterator = typename HoldList::iterator;

			TimeSeries(const int hold_data_size = 32);
			TimeSeries(const TimeSeries &) = delete;

			TimeSeries &operator = (const TimeSeries &);

			HoldType latest();

			//! old n ---- 0 new
			HoldType &at(const int &);

			HoldList get_raw(),
					 get_all();

			void set(const T &value);
			void set(const T &value, const typename HoldType::TimestampType &timestamp);

			void reset();
			void reset_timer();
			void zero_reset();

			int hold_size() const;

		private :
			int maximum_size_of_hold_list;

			HoldListIterator seek_iterator;

			//std::unique_ptr<TimeSeriesBuffer<T>> buffer;

			std::unique_ptr<std::mutex> mutex;

			std::unique_ptr<NanoSeconds> start_nanoseconds;

			std::unique_ptr<HoldList> hold_list;

			NanoSeconds get_current_nanoseconds();
	};
}

