
/**
  *
  * @file Messenger.hpp
  * @brief Message output processing
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include <unordered_map>
#include <fstream>
#include <memory>
#include <string>
#include <chrono>
#include <deque>
#include <vector>
#include <mutex>

#include "MessageLevels.hpp"
#include "../CommandLineArgumentReader.hpp"

namespace Tools {
	namespace Log {
		class Messenger final {
			public :
				Messenger(int argc, char **argv);
				~Messenger();

				Messenger &set_message_level_name(const MessageLevels &level, const std::string &name),
						  &set_output_message_level(const MessageLevels &level),
						  &disable_terminal_output(const bool &);

				size_t get_current_message_size();

				void print_once();

				void register_message(const MessageLevels &level, const std::string &message_str);

				template <typename T>
				void register_message(const MessageLevels &level, const std::vector<T> &message_vec);

			private :
				bool enable_printout_terminal,
					 enable_color_printout;

				std::chrono::microseconds current_microseconds;
				std::time_t current_time;

				std::string program_name,
							output_format_message;

				std::unique_ptr<std::mutex> mutex;

				MessageLevels output_level;

				std::unique_ptr<std::ofstream> logfile;

				std::deque<std::pair<MessageLevels, std::string>> messages;

				std::unordered_map<MessageLevels, std::string> level_names,
															   message_level_colors;

				std::unordered_map<std::string, std::string> colors;

				void settings_from_argument(CommandLineArgumentReader &),
					 open_logfile(CommandLineArgumentReader &),
					 enable_printout_log(CommandLineArgumentReader &),
					 set_message_threshold(CommandLineArgumentReader &),
					 enable_log_color(CommandLineArgumentReader &);

				void close();

				void default_settings(),
					 register_printout_colors(),
					 register_printout_level_colors(),
					 register_default_message_level_names();

				void write_out_message_to_terminal(const std::string &message),
					 write_out_message_to_logfile(const std::string &message);

				bool make_message(std::string &return_message_line, const bool &enable_color = true);

				void printout_begin_callback(),
					 printout_end_callback(),
					 printout_callback();
		};
	}
}

