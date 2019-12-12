
#pragma once

#include <stdexcept>
#include <memory>

namespace IO {
	namespace Device {
		template <typename CommnadController>
		class SerialDeviceBase {
			public :
				using CommandControllerPtr = std::shared_ptr<CommnadController>;

				virtual void register_controller(CommandControllerPtr &new_command_controller) {
					if(!new_command_controller) {
						std::runtime_error("Failed access CommandController from IO::Device::SerialDeviceBase");
					}

					command_controller = new_command_controller;
				}

			protected :
				CommandControllerPtr command_controller;

		};
	}
}

