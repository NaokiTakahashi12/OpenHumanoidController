
/**
  *
  * @file SerialDeviceSelector.hpp
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include "ObjectSelector.hpp"

#include <RobotStatus/Information.hpp>

namespace IO {
	template <typename Base>
	class SerialDeviceSelector final : public ObjectSelector<Base, std::string> {
		public :
			SerialDeviceSelector(RobotStatus::InformationPtr &);
			~SerialDeviceSelector();

		private :
			RobotStatus::InformationPtr robo_info;

			void default_register();
	};
}
