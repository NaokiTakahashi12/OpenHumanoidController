
/**
  *
  * @file Kondo.hpp
  * @authors Yasuo Hayashibara
  *		     Naoki Takahashi
  *
  **/

#pragma once

#include "SerialControllerBase.hpp"

namespace IO {
	namespace Communicator {
		namespace SerialController {
			class Kondo final : public SerialControllerBase {
				public :
					Kondo();
					~Kondo();

					static std::string get_key();

					void launch() override;
					void async_launch() override;

				private :
					ParseFunction create_data_parser() override;
					bool packet_splitter(const ReadBuffer &, const Length &);

			};
		}
	}
}

