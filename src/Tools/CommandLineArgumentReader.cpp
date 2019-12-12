
#include "CommandLineArgumentReader.hpp"

namespace Tools {
	CommandLineArgumentReader::CommandLineArgumentReader(int argc, char **argv) {
		number_of_argument = argc;

		for(int i = 0; i < number_of_argument; i ++) {
			arguments.push_back(argv[i]);
		}
	}

	std::string CommandLineArgumentReader::get_head() {
		return arguments.front();
	}

	std::string CommandLineArgumentReader::get(const std::string &identity) {
		size_t number_of_hit_argument = 0;

		for(auto &&arg : arguments) {
			if(arg == identity) {
				if(number_of_hit_argument < arguments.size() - 1) {
					return arguments.at(number_of_hit_argument + 1);
				}
				else {
					throw std::runtime_error("Argument over size error from Tools::CommandLineArgumentReader::get(identity)");
				}
			}
			number_of_hit_argument ++;
		}
		return "";
	}

	std::vector<std::string> CommandLineArgumentReader::get() {
		return arguments;
	}
}

