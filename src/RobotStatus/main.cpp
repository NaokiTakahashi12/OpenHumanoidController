
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

		for(auto &&[value, timestamp] : time_series->get_raw()) {
			std::cout << value << std::endl;
			std::cout << timestamp << std::endl;
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

		Tools::Math::Vector3<float> debug_accel_data;
		debug_accel_data << 0,0,1;
		for(auto i = 0; i < 10; i ++) {
			robo_info->accelerometers_data->set(debug_accel_data);
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			robo_info->accelerometers_data->set(debug_accel_data);
			std::cout << (robo_info->accelerometers_data->at(0).timestamp - robo_info->accelerometers_data->at(1).timestamp) * 0.000001 << " ms" << std::endl;
		}
	}
	catch(const std::exception &error) {
		std::cerr << error.what() << std::endl;
	}
}

