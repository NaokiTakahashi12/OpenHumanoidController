
/**
  *
  * @file Messenger.cpp
  * @author Naoki Takahashi
  *
  **/

#include "Messenger.hpp"

#include <cmath>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace Tools {
	namespace Log {
		Messenger::Messenger(int argc, char **argv) {
			CommandLineArgumentReader reader(argc, argv);

			enable_printout_terminal = true;
			program_name = reader.get_head();
			mutex = std::make_unique<std::mutex>();
			default_settings();
			set_output_message_level(MessageLevels::trace);
			settings_from_argument(reader);
			printout_begin_callback();
		}

		Messenger::~Messenger() {
			printout_end_callback();
			close();
		}

		Messenger &Messenger::set_message_level_name(const MessageLevels &level, const std::string &name) {
			level_names[level] = name;
			return *this;
		}

		Messenger &Messenger::set_output_message_level(const MessageLevels &level) {
			output_level = level;
			return *this;
		}

		Messenger &Messenger::disable_terminal_output(const bool &disable_terminal) {
			enable_printout_terminal = !disable_terminal;
			return *this;
		}

		size_t Messenger::get_current_message_size() {
			return messages.size();
		}

		void Messenger::print_once() {
			if(make_message(output_format_message, enable_color_printout)) {
				write_out_message_to_terminal(output_format_message);
				write_out_message_to_logfile(output_format_message);
			}
		}

		void Messenger::register_message(const MessageLevels &level, const std::string &message_str) {
			std::lock_guard<std::mutex> lock(*mutex);
			if(output_level <= level) {
				messages.emplace_back(make_pair(level, message_str));
			}
		}

		template <typename T>
		void Messenger::register_message(const MessageLevels &level, const std::vector<T> &message_vec) {
			std::lock_guard<std::mutex> lock(*mutex);
			if(output_level > level) {
				return;
			}
			std::stringstream ss;
			for(auto &&msg : message_vec) {
				ss << msg << ", ";
			}
			messages.emplace_back(make_pair(level, ss.str()));
		}

		void Messenger::settings_from_argument(CommandLineArgumentReader &reader) {
			open_logfile(reader);
			enable_printout_log(reader);
			set_message_threshold(reader);
			enable_log_color(reader);
		}

		void Messenger::open_logfile(CommandLineArgumentReader &reader) {
			constexpr auto log_file_identity = "--logfile";
			const auto filename = reader.get(log_file_identity);
			if(filename.empty()) {
				return;
			}
			close();
			logfile = std::make_unique<std::ofstream>(filename);
			if(!logfile->is_open()) {
				throw std::runtime_error("Can not open file from Tools::Log::Messenger");
			}
		}

		void Messenger::enable_printout_log(CommandLineArgumentReader &reader) {
			constexpr auto print_flag_identity = "--print",
						   disable_flag_identity = "false";
			const auto print_flag = reader.get(print_flag_identity);
			if(print_flag.empty()) {
				return;
			}
			if(0 == print_flag.compare(disable_flag_identity)) {
				enable_printout_terminal = false;
			}
			else {
				enable_printout_terminal = true;
			}
		}

		void Messenger::set_message_threshold(CommandLineArgumentReader &reader) {
			constexpr auto message_threshold_identity = "--message-threshold";
			const auto message_threshold_key = reader.get(message_threshold_identity);
			for(auto &&[key, string] : level_names) {
				if(string == message_threshold_key) {
					set_output_message_level(key);
					std::cout << string << " " << message_threshold_key << std::endl;
				}
			}
		}

		void Messenger::enable_log_color(CommandLineArgumentReader &reader) {
			constexpr auto enable_color_identity = "--enable-color",
					  	   enable_flag_identity = "true";
			const auto color_flag = reader.get(enable_color_identity);
			if(color_flag.empty()) {
				return;
			}
			if(0 == color_flag.compare(enable_flag_identity)) {
				enable_color_printout = true;
			}
			else {
				enable_color_printout = false;
			}
		}

		void Messenger::close() {
			if(logfile) {
				logfile->close();
			}
		}

		void Messenger::default_settings() {
			register_default_message_level_names();
			register_printout_colors();
			register_printout_level_colors();
		}

		void Messenger::register_printout_colors() {
			colors["reset"] = "\x1B[0m";
			colors["red"] = "\x1B[31m";
			colors["green"] = "\x1B[32m";
			colors["yellow"] = "\x1B[33m";
			colors["blue"] = "\x1B[34m";
			colors["magenta"] = "\x1B[35m";
			colors["cyan"] = "\x1B[36m";
			colors["white"] = "\x1B[36m";
		}

		void Messenger::register_printout_level_colors() {
			message_level_colors[MessageLevels::trace] = colors["blue"];
			message_level_colors[MessageLevels::debug] = colors["cyan"];
			message_level_colors[MessageLevels::info] = colors["reset"];
			message_level_colors[MessageLevels::warning] = colors["yellow"];
			message_level_colors[MessageLevels::error] = colors["magenta"];
			message_level_colors[MessageLevels::fatal] = colors["red"];
		}

		void Messenger::register_default_message_level_names() {
			level_names[MessageLevels::trace] 	= "Trace";
			level_names[MessageLevels::debug] 	= "Debug";
			level_names[MessageLevels::info] 	= "Info";
			level_names[MessageLevels::warning] = "Warning";
			level_names[MessageLevels::error] = "Error";
			level_names[MessageLevels::fatal] = "Fatal";
		}

		void Messenger::write_out_message_to_terminal(const std::string &message) {
			if(enable_printout_terminal) {
				std::cout << message;
			}
		}

		void Messenger::write_out_message_to_logfile(const std::string &message) {
			if(logfile) {
				*logfile << message;
			}
		}

		bool Messenger::make_message(std::string &return_message, const bool &enable_color) {
			std::lock_guard<std::mutex> lock(*mutex);
			static bool called = false;
			if(messages.empty()) {
				return false;
			}
			std::stringstream message_stream;
			auto message = messages.front();
			messages.pop_front();

			current_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
			current_microseconds = std::chrono::duration_cast<std::chrono::microseconds>(
					std::chrono::system_clock::now().time_since_epoch()
			);

			if(called) {
				message_stream << ",";
			}
			message_stream << "\n{\"Time\":\"" 
						   << std::put_time(localtime(&current_time), "%F %T")
						   << "."
						   << std::setfill('0') << std::setw(log10(std::mega::num))
						   << current_microseconds.count() % std::mega::num
						   //<< " UTC" << std::put_time(localtime(&current_time), "%z")
						   << "\"," << "\"Exe\":\"" << program_name << "\","
						   << "\"Level\":\"";
			if(enable_color) {
				message_stream << message_level_colors[message.first];
			}
			message_stream << level_names[message.first];
			if(enable_color) {
				message_stream << colors["reset"];
			}
			message_stream << "\",\"Message\":\"" << message.second << "\" }";
			return_message = message_stream.str();

			called = true;
			return true;
		}

		void Messenger::printout_begin_callback() {
			std::string output_format_message;
			output_format_message = "{\"Log\":[";
			write_out_message_to_terminal(output_format_message);
			write_out_message_to_logfile(output_format_message);
		}

		void Messenger::printout_end_callback() {
			std::string output_format_message;
			output_format_message = "\n]}\n";
			write_out_message_to_terminal(output_format_message);
			write_out_message_to_logfile(output_format_message);
		}

		template void Messenger::register_message<char>(const MessageLevels &, const std::vector<char> &);
		template void Messenger::register_message<uint8_t>(const MessageLevels &, const std::vector<uint8_t> &);
		template void Messenger::register_message<uint32_t>(const MessageLevels &, const std::vector<uint32_t> &);
		template void Messenger::register_message<uint64_t>(const MessageLevels &, const std::vector<uint64_t> &);
		template void Messenger::register_message<int>(const MessageLevels &, const std::vector<int> &);
		template void Messenger::register_message<float>(const MessageLevels &, const std::vector<float> &);
		template void Messenger::register_message<double>(const MessageLevels &, const std::vector<double> &);
	}
}

