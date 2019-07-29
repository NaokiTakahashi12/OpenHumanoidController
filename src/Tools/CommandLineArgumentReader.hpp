
#pragma once

#include <exception>
#include <stdexcept>
#include <vector>
#include <string>

namespace Tools {
	class CommandLineArgumentReader {
		public :
			CommandLineArgumentReader(int argc, char **argv);
			CommandLineArgumentReader(const CommandLineArgumentReader &) = delete;

			std::string get_head();

			std::string get(const std::string &identity);
			std::vector<std::string> get();

		private :
			int number_of_argument;
			std::vector<std::string> arguments;

	};
}

