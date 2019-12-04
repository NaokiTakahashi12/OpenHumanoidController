
/**
  *
  * @file ObjectSelector.hpp
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include <memory>
#include <unordered_map>
#include <stdexcept>

namespace IO {
	template <class Base, typename Key>
	class ObjectSelector {
		public :
			using BasePtr = std::unique_ptr<Base>;
			using EntryObject = std::pair<Key, BasePtr>;

			void operator () (EntryObject pair) {
				if(is_registered(pair.first)) {
					erase(pair.first);
				}

				register_element_of_map(pair.first, pair.second);
			}

			void clear() {
				object_map.clear();
			}

			void erase(const Key &key) {
				object_map.erase(key);
			}

			void register_object(const Key &key, BasePtr base_pointer) {
				is_registered_assert(key);
				register_element_of_map(key, base_pointer);
			}

			void register_object(EntryObject pair) {
				is_registered_assert(pair.first);
				register_element_of_map(pair.first, pair.second);
			}

			void re_register_object(const Key &key, BasePtr base_pointer) {
				if(is_registered(key)) {
					erase(key);
				}

				register_element_of_map(key, base_pointer);
			}

			void re_register_object(EntryObject pair) {
				if(is_registered(pair.first)) {
					erase(pair.first);
				}

				register_element_of_map(pair.first, pair.second);
			}

			bool is_registered(const Key &key) const {
				return object_map.find(key) != object_map.cend();
			}

			BasePtr choice_object(const Key &key) {
				if(!is_registered(key)) {
					throw std::runtime_error("Could not select key from Kinematics::ObjectSelector");
				}

				auto p = std::move(object_map[key]);

				erase(key);

				return std::move(p);
			}

			float load_factor() const {
				return object_map.load_factor();
			}

		protected :
			using ObjectMap = std::unordered_map<Key, BasePtr>;

			ObjectMap object_map;

		private :
			void register_element_of_map(const Key &key, BasePtr &base_pointer) {
				if(!base_pointer) {
					throw std::runtime_error("Failed access to base_pointer from Kinematics::ObjectSelector");
				}

				object_map[key] = std::move(base_pointer);
			}

			void is_registered_assert(const Key &key) {
				if(is_registered(key)) {
					throw std::runtime_error("Failed register already registered base_pointer from Kinematics::ObjectSelector");
				}
			}
	};
}

