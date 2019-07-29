
/**
  *
  * @file TimeSeriesData.hpp
  * @brief Timed value class
  * @auther Naoki Takahashi
  *
  **/

#pragma once

#include <cstdint>

namespace RobotStatus {
	template <typename T>
	struct TimeSeriesData {
		public :
			using TimestampType = uint64_t;

			T value;
			TimestampType timestamp;

			TimeSeriesData &operator = (const TimeSeriesData &);
			bool operator == (const TimeSeriesData &) const;
			bool operator != (const TimeSeriesData &) const;
			bool operator <= (const TimeSeriesData &) const;
			bool operator >= (const TimeSeriesData &) const;
			bool operator < (const TimeSeriesData &) const;
			bool operator > (const TimeSeriesData &) const;
	};
}

