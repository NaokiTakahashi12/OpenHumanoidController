
/**
  *
  * @file AngularMomentum.cpp
  * @author Naoki Takahashi
  *
  **/

#include "AngularMomentum.hpp"

namespace Kinematics {
	namespace Quantity {
		template <typename Scaler>
		AngularMomentum<Scaler>::AngularMomentum() {
			reset();
		}

		template <typename Scaler>
		AngularMomentum<Scaler>::~AngularMomentum() {
		}

		template <typename Scaler> 
		void AngularMomentum<Scaler>::reset() {
			change_of_angular_momentum = Vector3::Zero();
			angular_momentum_vector = Vector3::Zero();
		}

		template <typename Scaler>
		const typename AngularMomentum<Scaler>::Vector3 &AngularMomentum<Scaler>::change() const {
			return change_of_angular_momentum;
		}

		template <typename Scaler>
		typename AngularMomentum<Scaler>::Vector3 &AngularMomentum<Scaler>::change() {
			return change_of_angular_momentum;
		}

		template <typename Scaler>
		const typename AngularMomentum<Scaler>::Vector3 &AngularMomentum<Scaler>::operator () () const {
			return angular_momentum_vector;
		}

		template <typename Scaler>
		typename AngularMomentum<Scaler>::Vector3 &AngularMomentum<Scaler>::operator () () {
			return angular_momentum_vector;
		}

		template class AngularMomentum<float>;
		template class AngularMomentum<double>;
		template class AngularMomentum<long double>;
	}
}

