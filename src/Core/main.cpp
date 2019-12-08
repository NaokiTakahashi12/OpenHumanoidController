
#include <iostream>
#include <exception>

#include "Launcher.hpp"

int main(int argc, char **argv) {
	try {
		Core::Launcher launcher(argc, argv);
	}
	catch(const std::exception &error) {
		std::cerr << error.what() << std::endl;
	}
}

