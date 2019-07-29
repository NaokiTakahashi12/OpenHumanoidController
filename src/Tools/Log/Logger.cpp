
/**
  *
  * @file Logger.cpp
  * @author Naoki Takahashi
  *
  **/

#include "Logger.hpp"

namespace Tools {
	namespace Log {
		Logger::Logger() {
		}
		
		Logger::Logger(int argc, char **argv) {
			messenger = std::make_unique<Messenger>(argc, argv);
		}

		Logger::~Logger() {
			close_loger_thread();
		}

		Logger &Logger::disable_print_terminal(const bool &disable_print_out) {
			if(messenger) {
				messenger->disable_terminal_output(disable_print_out);
			}
			return *this;
		}

		Logger &Logger::output_message_level(const MessageLevels &level) {
			if(messenger) {
				messenger->set_output_message_level(level);
			}
			return *this;
		}

		void Logger::message(const MessageLevels &level, const std::string &message_str) {
			if(messenger) {
				messenger->register_message(level, message_str);
			}
		}

		void Logger::message(const MessageLevels &level, const char *message_str) {
			if(messenger) {
				messenger->register_message(level, message_str);
			}
		}

		template <typename T>
		void Logger::message(const MessageLevels &level, const T &message_tem) {
			if(messenger) {
				messenger->register_message(level, std::to_string(message_tem));
			}
		}

		template <typename T>
		void Logger::message(const MessageLevels &level, const std::vector<T> &message_vec) {
			if(messenger) {
				messenger->register_message(level, message_vec);
			}
		}

		template <typename T, int R, int C>
		void Logger::message(const MessageLevels &level, const Math::Matrix<T, R, C> &matrix) {
			if(messenger) {
				for(auto i = 0; i < C; i++) {
					std::stringstream ss;
					ss << "[" << matrix.block(0, i, R, 1).transpose() << "]";
					messenger->register_message(level, ss.str());
				}
				messenger->register_message(level, "");
			}
		}

		template <typename T, int C>
		void Logger::message(const MessageLevels &level, const Math::Vector<T, C> &vector) {
			if(messenger) {
				std::stringstream ss;
				ss << "[" << vector.transpose() << "]";
				messenger->register_message(level, ss.str());
				messenger->register_message(level, "");
			}
		}

		void Logger::start_loger_thread() {
			if(!messenger) {
				return;
			}
			if(!message_thread) {
				enable_close = false;
				message_thread = std::make_unique<std::thread>(&Logger::print_message, this);
			}
			else {
				throw std::logic_error("Started loger thread.");
			}
		}

		void Logger::close_loger_thread() {
			if(!messenger) {
				return;
			}
			while(messenger->get_current_message_size() > 0) {}
			if(message_thread) {
				enable_close = true;
				message_thread->join();
				message_thread.reset();
			}
		}

		void Logger::print_message() {
			while(!enable_close) {
				messenger->print_once();
				std::this_thread::yield();
			}
		}

		template void Logger::message<char>(const MessageLevels &, const char &);

		template void Logger::message<uint8_t>(const MessageLevels &, const uint8_t &);
		template void Logger::message<uint32_t>(const MessageLevels &, const uint32_t &);
		template void Logger::message<uint64_t>(const MessageLevels &, const uint64_t &);
		template void Logger::message<int>(const MessageLevels &, const int &);
		template void Logger::message<float>(const MessageLevels &, const float &);
		template void Logger::message<double>(const MessageLevels &, const double &);

		template void Logger::message<char>(const MessageLevels &, const std::vector<char> &);

		template void Logger::message<uint8_t>(const MessageLevels &, const std::vector<uint8_t> &);
		template void Logger::message<uint32_t>(const MessageLevels &, const std::vector<uint32_t> &);
		template void Logger::message<uint64_t>(const MessageLevels &, const std::vector<uint64_t> &);
		template void Logger::message<int>(const MessageLevels &, const std::vector<int> &);
		template void Logger::message<float>(const MessageLevels &, const std::vector<float> &);
		template void Logger::message<double>(const MessageLevels &, const std::vector<double> &);

		template void Logger::message<float, 2, 2>(const MessageLevels &, const Math::Matrix2<float> &);
		template void Logger::message<float, 3, 3>(const MessageLevels &, const Math::Matrix3<float> &);
		template void Logger::message<float, 4, 4>(const MessageLevels &, const Math::Matrix4<float> &);
		template void Logger::message<double, 2, 2>(const MessageLevels &, const Math::Matrix2<double> &);
		template void Logger::message<double, 3, 3>(const MessageLevels &, const Math::Matrix3<double> &);
		template void Logger::message<double, 4, 4>(const MessageLevels &, const Math::Matrix4<double> &);

		template void Logger::message<float, 2>(const MessageLevels &, const Math::Vector2<float> &);
		template void Logger::message<float, 3>(const MessageLevels &, const Math::Vector3<float> &);
		template void Logger::message<float, 4>(const MessageLevels &, const Math::Vector4<float> &);
		template void Logger::message<double, 2>(const MessageLevels &, const Math::Vector2<double> &);
		template void Logger::message<double, 3>(const MessageLevels &, const Math::Vector3<double> &);
		template void Logger::message<double, 4>(const MessageLevels &, const Math::Vector4<double> &);
	}
}

