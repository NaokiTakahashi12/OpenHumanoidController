
#pragma once

#include "SerialControllerBase.hpp"

namespace IO {
	namespace Communicator {
		namespace SerialController {
			class Simple final : public SerialControllerBase {
				public :
					Simple();
					~Simple();

					void launch() override,
						 async_launch() override;
			};
		}
	}
}

