
/**
  *
  * @file VMU931.cpp
  * @author Naoki Takahashi
  *
  **/

#include "VMU931.hpp"

#include <stdexcept>

#include <Tools/Converter/Endian.hpp>

namespace IO {
	namespace Device {
		namespace Sensor {
			namespace IMU {
				VMU931::VMU931(RobotStatus::InformationPtr &robot_status_information) : InertialMeasurementUnit(robot_status_information) {
					this->robo_info->create_quat_data_space();
					this->robo_info->create_euler_data_space();
				}

				VMU931::~VMU931() {
				}

				std::string VMU931::get_key() {
					return "VMU931";
				}

				void VMU931::enable(const VMU931::Streams &streams) {
					std::string stream(streams_prefix);
					stream.append({static_cast<char>(streams)});

					if(streaming_list.find(stream)) {
						streaming_list.append(stream);
						create_data_space(streams);
					}
					else {
						throw std::runtime_error("Already exist stream list from IO::Device::Sensor::IMU::VMU931");
					}
				}

				void VMU931::enable_all() {
					for(auto &&s : {
						Streams::Accelerometers,
						Streams::Gyroscopes,
						Streams::Magnetometers,
						Streams::Heading
					}) {
						enable(s);
					}
				}

				void VMU931::launch() {
					if(!this->command_controller) {
						throw std::runtime_error("Not exist serial controller pointer from IO::Device::Sensor::IMU::VMU931");
					}

					this->command_controller->register_parse(
							[this](const ReadBuffer &read_buffer, const Length &bytes) {
								return packet_splitter(read_buffer, bytes);
							}
					);
					this->command_controller->set_packet(streaming_list);

					this->command_controller->launch();
				}

				void VMU931::async_launch() {
					if(!this->command_controller) {
						throw std::runtime_error("Not exist serial controller pointer from IO::Device::Sensor::IMU::VMU931");
					}

					this->command_controller->register_parse(
							[this](const ReadBuffer &read_buffer, const Length &bytes) {
								return packet_splitter(read_buffer, bytes);
							}
					);
					this->command_controller->set_packet(streaming_list);

					this->command_controller->async_launch();
				}

				void VMU931::create_data_space(const Streams &stream) {
					switch(stream) {
						case Streams::Accelerometers :
							this->robo_info->create_accel_data_space();
							break;
						case Streams::Gyroscopes :
							this->robo_info->create_gyro_data_space();
							break;
						case Streams::Magnetometers :
							this->robo_info->create_magnet_data_space();
							break;
						case Streams::EulerAngles :
							this->robo_info->create_euler_data_space();
							break;
						case Streams::Quaternions :
							this->robo_info->create_quat_data_space();
							break;
						case Streams::Heading :
							this->robo_info->create_head_data_space();
							break;
					}
				}

				template <>
				uint32_t &VMU931::single_value_cast<uint32_t>(const ReadBuffer &read_buffer, const uint8_t &position) {
					static uint32_t value;

					value = *reinterpret_cast<const uint32_t *>(read_buffer.data() + position);
					value = Tools::Converter::Endian::swap(value);

					return value;
				}

				template <>
				uint64_t &VMU931::single_value_cast<uint64_t>(const ReadBuffer &read_buffer, const uint8_t &position) {
					static uint64_t value;

					value = *reinterpret_cast<const uint64_t *>(single_value_cast<uint32_t>(read_buffer, position));

					return value;
				}

				template <>
				float &VMU931::single_value_cast<float>(const ReadBuffer &read_buffer, const uint8_t &position) {
					static float value;

					value = *reinterpret_cast<const float *>(&single_value_cast<uint32_t>(read_buffer, position));

					return value;
				}

				template <>
				uint32_t &VMU931::timestamp_parser<uint32_t>(const ReadBuffer &read_buffer, const uint8_t &head_position) {
					return single_value_cast<uint32_t>(read_buffer, head_position + 3);
				}

				template <>
				uint64_t &VMU931::timestamp_parser<uint64_t>(const ReadBuffer &read_buffer, const uint8_t &head_position) {
					static uint64_t value;

					value = timestamp_parser<uint32_t>(read_buffer, head_position) * std::mega::num;
					
					return value;
				}

				float &VMU931::heading_parser(const ReadBuffer &read_buffer, const uint8_t &head_position) {
					static float value;

					value = single_value_cast<float>(read_buffer, head_position + 7);

					return value;
				}

				Tools::Math::Vector3<float> &VMU931::vector3_parser(const ReadBuffer &read_buffer, const uint8_t &head_position) {
					static Tools::Math::Vector3<float> vector;

					vector.x() = single_value_cast<float>(read_buffer, head_position + 7);
					vector.y() = single_value_cast<float>(read_buffer, head_position + 11);
					vector.z() = single_value_cast<float>(read_buffer, head_position + 15);

					return vector;
				}

				Tools::Math::Vector4<float> &VMU931::vector4_parser(const ReadBuffer &read_buffer, const uint8_t &head_position) {
					static Tools::Math::Vector4<float> vector;

					vector(0) = single_value_cast<float>(read_buffer, head_position + 7);
					vector(1) = single_value_cast<float>(read_buffer, head_position + 11);
					vector(2) = single_value_cast<float>(read_buffer, head_position + 15);
					vector(3) = single_value_cast<float>(read_buffer, head_position + 19);

					return vector;
				}

				void VMU931::packet_parse(const ReadBuffer &read_buffer, const uint8_t &head_position) {
					constexpr auto head_size = 2;
					switch(static_cast<Streams>(read_buffer.at(head_position + head_size))) {
						case Streams::Accelerometers :
							this->robo_info->accelerometers_data->set(
									vector3_parser(read_buffer, head_position),
									timestamp_parser<uint64_t>(read_buffer, head_position)
							);
							break;
						case Streams::Gyroscopes :
							this->robo_info->gyroscopes_data->set(
									vector3_parser(read_buffer, head_position),
									timestamp_parser<uint64_t>(read_buffer, head_position)
							);
							break;
						case Streams::Magnetometers :
							this->robo_info->magnetometers_data->set(
									vector3_parser(read_buffer, head_position),
									timestamp_parser<uint64_t>(read_buffer, head_position)
							);
							break;
						case Streams::EulerAngles :
							this->robo_info->eulerangles_data->set(
									vector3_parser(read_buffer, head_position),
									timestamp_parser<uint64_t>(read_buffer, head_position)
							);
							break;
						case Streams::Quaternions :
							this->robo_info->quaternions_data->set(
									vector4_parser(read_buffer, head_position),
									timestamp_parser<uint64_t>(read_buffer, head_position)
							);
							break;
						case Streams::Heading :
							this->robo_info->heading_data->set(
									heading_parser(read_buffer, head_position),
									timestamp_parser<uint64_t>(read_buffer, head_position)
							);
							break;
					}
				}

				bool VMU931::packet_splitter(const ReadBuffer &read_buffer, const Length &bytes) {
					static uint8_t head_of_processing_packet = 0;
					if(!broken_packet_checker(read_buffer, bytes, head_of_processing_packet)) {
						head_of_processing_packet += read_buffer.at(head_of_processing_packet + 1);
						packet_splitter(read_buffer, bytes);
					}
					else {
						head_of_processing_packet = 0;
						return false;
					}
					return true;
				}

				bool VMU931::broken_packet_checker(const ReadBuffer &read_buffer, const Length &bytes, const uint8_t &head_position) {
					if(bytes >= IO::Communicator::SerialFlowScheduler::maximum_read_buffer) {
						return true;
					}
					else if(read_buffer.at(head_position + 1) > bytes) {
						return true;
					}
					return constant_bit_checker(read_buffer, bytes, head_position);
				}

				bool VMU931::constant_bit_checker(const ReadBuffer &read_buffer, const Length &bytes, const uint8_t &head_position) {
					if(read_buffer.at(head_position) != data_start_bit) {
						return true;
					}
					else if(read_buffer.at(head_position + read_buffer.at(head_position + 1) - 1) != data_end_bit) {
						return true;
					}
					else {
						packet_parse(read_buffer, head_position);
					}
					return end_packet_checker(read_buffer, bytes, head_position);
				}

				bool VMU931::end_packet_checker(const ReadBuffer &read_buffer, const Length &bytes, const uint8_t &head_position) {
					if((read_buffer.at(head_position + 1) + head_position) >= bytes ) {
						return true;
					}
					return false;
				}
			}
		}
	}
}

