
/**
  * 
  * @file JsonLoader.hpp
  * @author Naoki Takahashi
  * 
  **/

#pragma once

#include <vector>
#include <string>

#include <boost/property_tree/ptree.hpp>

namespace Tools {
	namespace ConfigFileOperator {
		class JsonLoader {
			public :
				JsonLoader(const std::string &file_name);

				JsonLoader &operator = (const JsonLoader &);
				bool operator == (const JsonLoader &) const;

				JsonLoader &load_json_filename(const std::string &file_name);

				bool empty() const;

				std::string get_filename() const;

				template <typename T>
				T get_parameter(const std::string &value_name);

				template <typename T>
				std::vector<T> get_parameter_tree(const std::string &tree_name, const std::string &value_name); 

				template <typename T>
				std::vector<T> get_parameter_vector(const std::string &value_name);

				template <typename T>
				std::vector<std::vector<T>> get_parameter_tree_vector(const std::string &tree_name, const  std::string &value_name);

			private :
				bool opened_jsonfile;

				std::string json_filename;

				boost::property_tree::ptree jsonfile;

				boost::property_tree::ptree open_json_file(const std::string &file_name);

				void check_opened_jsonfile();
		};
	}
}

