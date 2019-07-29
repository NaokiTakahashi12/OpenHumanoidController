
#include <iostream>
#include <exception>
#include <chrono>
#include <memory>
#include <thread>

#include <Tools/Math/Matrix.hpp>

#include "Information.hpp"
#include "TimeSeries.hpp"

int main() {
	try {
		auto time_series = std::make_unique<RobotStatus::TimeSeries<Tools::Math::MatrixX<double>>>(8);
		auto start_point = std::chrono::high_resolution_clock::now();
		auto end_point = std::chrono::high_resolution_clock::now();
		int n = 1000;


		std::cout << "RobotStatus::TimeSeries Test" << std::endl;
		// Access time checker
		start_point = std::chrono::high_resolution_clock::now();
		for(auto i = 0; i < n; i++) {
			time_series->set(Tools::Math::Matrix3<double>::Constant(i));
		}
		end_point = std::chrono::high_resolution_clock::now();
		std::cout << "Set time is " << std::chrono::duration<float, std::milli>(end_point - start_point).count() << " ms" << std::endl;

		auto matrix = Tools::Math::Matrix3<double>();
		start_point = std::chrono::high_resolution_clock::now();
		for(auto i = 0; i < n; i++) {
			matrix = Tools::Math::Matrix3<double>::Constant(i);
		}
		end_point = std::chrono::high_resolution_clock::now();
		std::cout << "Single Matrix time is " << std::chrono::duration<float, std::milli>(end_point - start_point).count() << " ms" << std::endl;

		start_point = std::chrono::high_resolution_clock::now();
		time_series->get_all();
		end_point = std::chrono::high_resolution_clock::now();
		std::cout << "Sort time is " << std::chrono::duration<float, std::milli>(end_point - start_point).count() << " ms" << std::endl;

		for(auto &&ts : time_series->get_raw()) {
			std::cout << ts.value << std::endl;
			std::cout << ts.timestamp << std::endl;
		}
		std::cout << std::endl;

		start_point = std::chrono::high_resolution_clock::now();
		for(auto i = 0, max_size = 8; i < max_size; i++) {
			time_series->at(i);
		}
		end_point = std::chrono::high_resolution_clock::now();
		std::cout << "Random access time is " << std::chrono::duration<float, std::milli>(end_point - start_point).count() << " ms" << std::endl;
		std::cout << time_series->at(0).value << std::endl;
		std::cout << time_series->at(1).value << std::endl;
		std::cout << time_series->at(2).value << std::endl;

		std::cout << "RobotStatus::Information Test" << std::endl;
		auto robo_info = std::make_unique<RobotStatus::Information>();
		robo_info->set_config_filename<RobotStatus::Information::RobotType::Humanoid>("robot.conf");
		std::cout << robo_info->get_config_filename<RobotStatus::Information::RobotType::Humanoid>() << std::endl;
		robo_info->create_accel_data_space();
	}
	catch(const std::exception &error) {
		std::cerr << error.what() << std::endl;
	}
}

