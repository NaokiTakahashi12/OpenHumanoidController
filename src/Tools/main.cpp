
/**
  *
  * @file main.cpp
  * @brief Test main function
  * @author Naoki Takahashi
  *
  **/

#include <exception>
#include <iostream>
#include <string>
#include <bitset>
#include <chrono>
#include <thread>

#include <boost/asio.hpp>

#include "Constant.hpp"

#include "Converter/Endian.hpp"

#include "CommandLineArgumentReader.hpp"

#include "Byte.hpp"

#include "Log/Logger.hpp"

#include "Math/Differential/Function.hpp"
#include "Math/Matrix.hpp"
#include "Math/Unit/SI.hpp"
#include "Math/Unit/UnofficialSI.hpp"
#include "Math/Unit/Value.hpp"
#include "Math/MatrixOperation.hpp"
#include "Math/MathLiterals.hpp"

int main(int argc, char **argv) {
	try {
		std::vector<double> log_vec;
		Tools::Log::Logger logger(argc, argv);

		if(Tools::Converter::Endian::is_big_endian()) {
			logger.message(Tools::Log::MessageLevels::info, "This computer is big endian");
		}
		else if(Tools::Converter::Endian::is_little_endian()) {
			logger.message(Tools::Log::MessageLevels::info, "This computer is little endian");
		}
		else {
			logger.message(Tools::Log::MessageLevels::info, "This computer is unknown endian");
		}

		auto start_p = std::chrono::high_resolution_clock::now();
		auto end_p = std::chrono::high_resolution_clock::now();

		start_p = std::chrono::high_resolution_clock::now();

		end_p = std::chrono::high_resolution_clock::now();
		logger.message(Tools::Log::MessageLevels::debug, "Milliseconds of ");
		logger.message(Tools::Log::MessageLevels::debug, std::chrono::duration<double, std::ratio<1, 1000>>(end_p - start_p).count());

		auto message_level_test = "Test: Message level";
		logger.message(Tools::Log::MessageLevels::trace, message_level_test);
		logger.message(Tools::Log::MessageLevels::debug, message_level_test);
		logger.message(Tools::Log::MessageLevels::info, message_level_test);
		logger.message(Tools::Log::MessageLevels::warning, message_level_test);
		logger.message(Tools::Log::MessageLevels::error, message_level_test);
		logger.message(Tools::Log::MessageLevels::fatal, message_level_test);

		logger.start_loger_thread();

		log_vec.push_back(1.2);
		log_vec.push_back(1.2);
		log_vec.push_back(1.2);
		log_vec.push_back(1.2);

		Tools::Math::Vector<float, 3> v;
		Tools::Math::Matrix<float, 3, 3> m;
		v << 1, 2, 1;
		m = v.asDiagonal();
		logger.message(Tools::Log::MessageLevels::debug, "Test: message matrix");
//		logger.message(Tools::Log::MessageLevels::debug, v);
//		logger.message(Tools::Log::MessageLevels::debug, m);

		start_p = std::chrono::high_resolution_clock::now();

		logger.message(Tools::Log::MessageLevels::debug, "Test: STL vector");
		logger.message(Tools::Log::MessageLevels::debug, log_vec);
		logger.message(Tools::Log::MessageLevels::debug, "");

		end_p = std::chrono::high_resolution_clock::now();
		logger.message(Tools::Log::MessageLevels::debug, "Milliseconds of ");
		logger.message(Tools::Log::MessageLevels::debug, std::chrono::duration<double, std::ratio<1, 1000>>(end_p - start_p).count());

		auto dfunc = std::make_unique<Tools::Math::Differential::Function<float>>();

		Tools::Math::Unit::Value<float, Tools::Math::Unit::SI::metre> have_unit_value(0.1);

		short value_for_byte_debug = 360;
		std::string print_message;
		logger.message(Tools::Log::MessageLevels::trace, "Byte operation test");
		print_message = "Original data is " + std::to_string(value_for_byte_debug);
		logger.message(Tools::Log::MessageLevels::trace, print_message);
		auto low_byte = static_cast<uint8_t>(Tools::Byte::low_byte(value_for_byte_debug));
		print_message = "Low byte is " + std::to_string(low_byte);
		logger.message(Tools::Log::MessageLevels::trace, print_message);
		auto high_byte = static_cast<uint8_t>(Tools::Byte::high_byte(value_for_byte_debug));
		print_message = "High byte is " + std::to_string(high_byte);
		logger.message(Tools::Log::MessageLevels::trace, print_message);
		auto original_byte = Tools::Byte::union_byte(static_cast<short>(high_byte), static_cast<short>(low_byte));
		print_message = "Original byte is " + std::to_string(original_byte);
		logger.message(Tools::Log::MessageLevels::trace, print_message);

		logger.close_loger_thread();
	}
	catch(const std::exception &error) {
		std::cerr << error.what() << std::endl;
	}
	return 0;
}

