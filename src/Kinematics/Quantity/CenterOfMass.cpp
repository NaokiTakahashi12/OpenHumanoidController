
/**
  *
  * @file CenterOfMass.hpp
  * @author Naoki Takahashi
  *
  **/

#include "CenterOfMass.hpp"

namespace Kinematics {
	namespace Quantity {
		template <typename Scaler>
		CenterOfMass<Scaler>::CenterOfMass() {
			reset();
		}

		template <typename Scaler>
		CenterOfMass<Scaler>::~CenterOfMass() {
		}

		template <typename Scaler>
		void CenterOfMass<Scaler>::reset() {
			this->x = Vector3::Zero();
			this->dx = Vector3::Zero();
			this->ddx = Vector3::Zero();
		}

		template class CenterOfMass<float>;
		template class CenterOfMass<double>;
		template class CenterOfMass<long double>;
	}
}

