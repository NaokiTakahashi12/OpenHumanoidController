
/**
  *
  * @file SerialControlSelector.cpp
  * @author Naoki Takahashi
  *
  **/

#include "SerialControlSelector.hpp"

#include "SerialController/Simple.hpp"
#include "SerialController/Dynamixel.hpp"
#include "SerialController/Kondo.hpp"

namespace IO {
	namespace Communicator {
		SerialControlSelector::SerialControlSelector() {
			default_register();
		}

		SerialControlSelector::~SerialControlSelector() {
		}

		SerialControlSelector::SharedSerialController SerialControlSelector::choice_shared_object(const std::string &key) {
			SharedSerialController ret_ptr;

			ret_ptr = this->choice_object(key);

			return ret_ptr;
		}

		void SerialControlSelector::default_register() {
			this->register_object(
					{
						SerialController::Simple::get_key(),
						std::make_unique<SerialController::Simple>()
					}
			);
			this->register_object(
					{
						SerialController::Dynamixel::get_key(),
						std::make_unique<SerialController::Dynamixel>()
					}
			);
			this->register_object(
					{
						SerialController::Kondo::get_key(),
						std::make_unique<SerialController::Kondo>()
					}
			);
		}
	}
}

