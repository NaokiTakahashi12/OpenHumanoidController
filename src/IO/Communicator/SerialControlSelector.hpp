
/**
  *
  * @file SerialControlSelector.hpp
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include "../ObjectSelector.hpp"

#include "SerialController/SerialControllerBase.hpp"

namespace IO {
	namespace Communicator {
		class SerialControlSelector : public ObjectSelector<SerialController::SerialControllerBase, std::string> {
			public :
				using SharedSerialController = std::shared_ptr<SerialController::SerialControllerBase>;

				SerialControlSelector();
				~SerialControlSelector();

				SharedSerialController choice_shared_object(const std::string &key);

			private :
				void default_register();

		};
	}
}

