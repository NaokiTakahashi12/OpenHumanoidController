
/**
  *
  * @file JsonLoader.cpp
  * @author Naoki Takahashi
  *
  **/

#include "JsonLoader.hpp"

#include <exception>

#include <boost/property_tree/json_parser.hpp>
#include <boost/optional.hpp>
#include <boost/foreach.hpp>

namespace Tools {
	namespace ConfigFileOperator {
		JsonLoader::JsonLoader(const std::string &file_name) {
			opened_jsonfile = false;

			json_filename = file_name;

			jsonfile = open_json_file(file_name);
		}

		JsonLoader &JsonLoader::operator = (const JsonLoader &json_loader) {
			if(this != &json_loader) {
				this->opened_jsonfile = json_loader.opened_jsonfile;
				this->jsonfile = json_loader.jsonfile;
			}

			return *this;
		}

		bool JsonLoader::operator == (const JsonLoader &json_loader) const {
			return (this->jsonfile == json_loader.jsonfile);
		}

		JsonLoader &JsonLoader::load_json_filename(const std::string &file_name) {
			json_filename = file_name;

			jsonfile = open_json_file(file_name);

			return *this;
		}

		bool JsonLoader::empty() const {
			return jsonfile.empty();
		}

		std::string JsonLoader::get_filename() const {
			return json_filename;
		}

		template <typename T>
		T JsonLoader::get_parameter(const std::string &value_name) {
			if(value_name.size() == 0) {
				throw std::runtime_error("Not found parameter name");
			}

			check_opened_jsonfile();

			if(auto parameter = jsonfile.get_optional<T>(value_name)) {
				return parameter.get();
			}
			else {
				throw std::runtime_error("Parameter value name mismatch: " + value_name);
			}
		}

		template <typename T>
		std::vector<T> JsonLoader::get_parameter_tree(const std::string &tree_name, const std::string &value_name) {
			if(tree_name.size() == 0 || value_name.size() == 0) {
				throw std::runtime_error("Not found parameter tree name");
			}

			check_opened_jsonfile();

			std::vector<T> tree_hander;
			std::string full_value_name = tree_name + "." + value_name;

			BOOST_FOREACH(const boost::property_tree::ptree::value_type &type, jsonfile.get_child(tree_name.c_str())) {
				const auto &info = type.second;
				
				if(auto parameter = info.get_optional<T>(value_name.c_str())) {
					tree_hander.push_back(parameter.get());
				}
				else {
					throw std::runtime_error("Parameter value name mismatch: " + tree_name + " " + value_name);
				}
			}

			return tree_hander;
		}

		template <typename T>
		std::vector<T> JsonLoader::get_parameter_vector(const std::string &value_name) {
			if(value_name.size() == 0) {
				throw std::runtime_error("value name nothing");
			}

			check_opened_jsonfile();

			std::vector<T> vector_hander;
			T vector_element_hander;

			BOOST_FOREACH(const boost::property_tree::ptree::value_type &type, jsonfile.get_child(value_name.c_str())) {
				std::istringstream string2number(type.second.data());

				string2number >> vector_element_hander;

				vector_hander.push_back(vector_element_hander);
			}

			return vector_hander;
		}

		template <>
		std::vector<std::string> JsonLoader::get_parameter_vector(const std::string &value_name) {
			if(value_name.size() == 0) {
				throw std::runtime_error("value name nothing");
			}

			check_opened_jsonfile();

			std::vector<std::string> vector_hander;

			BOOST_FOREACH(const boost::property_tree::ptree::value_type &type, jsonfile.get_child(value_name.c_str())) {
				vector_hander.push_back(std::string(type.second.data()));
			}

			return vector_hander;
		}

		template <typename T>
		std::vector<std::vector<T>> JsonLoader::get_parameter_tree_vector(const std::string &tree_name, const std::string &value_name) {
			if(tree_name.size() == 0 || value_name.size() == 0) {
				throw std::runtime_error("tree name or value name nothing");
			}

			check_opened_jsonfile();

			std::vector<std::vector<T>> vector_hander;
			T vector_element_hander;

			BOOST_FOREACH(const boost::property_tree::ptree::value_type &type, jsonfile.get_child(tree_name.c_str())) {
				vector_hander.push_back(std::vector<T>());

				BOOST_FOREACH(const boost::property_tree::ptree::value_type &element_type, type.second.get_child(value_name.c_str())) {
					std::istringstream string2number(element_type.second.data());

					string2number >> vector_element_hander;

					vector_hander.back().push_back(vector_element_hander);
				}
			}

			return vector_hander;
		}

		template <>
		std::vector<std::vector<std::string>> JsonLoader::get_parameter_tree_vector(const std::string &tree_name, const std::string &value_name) {
			if(tree_name.size() == 0 || value_name.size() == 0) {
				throw std::runtime_error("tree name or value name nothing");
			}

			check_opened_jsonfile();

			std::vector<std::vector<std::string>> vector_hander;

			BOOST_FOREACH(const boost::property_tree::ptree::value_type &type, jsonfile.get_child(tree_name.c_str())) {
				vector_hander.push_back(std::vector<std::string>());

				BOOST_FOREACH(const boost::property_tree::ptree::value_type &element_type, type.second.get_child(value_name.c_str())) {
					vector_hander.back().push_back(std::string(element_type.second.data()));
				}
			}

			return vector_hander;
		}

		boost::property_tree::ptree JsonLoader::open_json_file(const std::string &file_name) {
			if(file_name.size() == 0) {
				throw std::runtime_error("Not found json filename");
			}

			boost::property_tree::ptree file;
			if(file_name.size() != 0) {
				read_json(file_name.c_str(), file);

				opened_jsonfile = true;
			}
			else if(file_name.size() == 0) {
				throw std::runtime_error("Set json filename error");
			}

			return file;
		}

		void JsonLoader::check_opened_jsonfile() {
			if(!opened_jsonfile) {
				throw std::runtime_error("Failed open jsonfile");
			}
		}

		template bool JsonLoader::get_parameter<bool>(const std::string &);
		template int JsonLoader::get_parameter<int>(const std::string &);
		template float JsonLoader::get_parameter<float>(const std::string &);
		template char JsonLoader::get_parameter<char>(const std::string &);
		template std::string JsonLoader::get_parameter<std::string>(const std::string &);

		template std::vector<bool> JsonLoader::get_parameter_tree<bool>(const std::string &, const std::string &);
		template std::vector<int> JsonLoader::get_parameter_tree<int>(const std::string &, const std::string &);
		template std::vector<float> JsonLoader::get_parameter_tree<float>(const std::string &, const std::string &);
		template std::vector<char> JsonLoader::get_parameter_tree<char>(const std::string &, const std::string &);
		template std::vector<std::string> JsonLoader::get_parameter_tree<std::string>(const std::string &, const std::string &);

		template std::vector<bool> JsonLoader::get_parameter_vector<bool>(const std::string &);
		template std::vector<int> JsonLoader::get_parameter_vector<int>(const std::string &);
		template std::vector<float> JsonLoader::get_parameter_vector<float>(const std::string &);

		template std::vector<std::vector<bool>> JsonLoader::get_parameter_tree_vector<bool>(const std::string &, const std::string &);
		template std::vector<std::vector<int>> JsonLoader::get_parameter_tree_vector<int>(const std::string &, const std::string &);
		template std::vector<std::vector<float>> JsonLoader::get_parameter_tree_vector<float>(const std::string &, const std::string &);
	}
}

