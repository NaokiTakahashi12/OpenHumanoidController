
/**
  *
  * @file Logger.hpp
  * @brief Logging class
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include "Messenger.hpp"

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "../Math/Matrix.hpp"

namespace Tools {
	namespace Log {
		class Logger {
			public :
				Logger();
				Logger(int argc, char **argv);

				virtual ~Logger();

				Logger &disable_print_terminal(const bool &),
					   &output_message_level(const MessageLevels &);

				void message(const MessageLevels &level, const std::string &message_str);
				void message(const MessageLevels &level, const char *message_str);

				template <typename T>
				void message(const MessageLevels &level, const T &message_tem);

				template <typename T>
				void message(const MessageLevels &level, const std::vector<T> &message_vec);

				template <typename T, int R, int C>
				void message(const MessageLevels &level, const Math::Matrix<T, R, C> &matrix);

				template <typename T, int C>
				void message(const MessageLevels &level, const Math::Vector<T, C> &vector);

				void start_loger_thread(),
					 close_loger_thread();

			private :
				bool enable_close;

				std::unique_ptr<std::thread> message_thread;

				std::unique_ptr<Messenger> messenger;

				void print_message();
		};

		using LoggerPtr = std::shared_ptr<Logger>;
	}
}

