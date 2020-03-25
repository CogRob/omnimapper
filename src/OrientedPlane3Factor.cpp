/*
 * OrientedPlane3Factor.cpp
 *
 *  Created on: Jan 29, 2014
 *      Author: Natesh Srinivasan
 */

#include <omnimapper/OrientedPlane3Factor.h>

using namespace std;

namespace omnimapper {

//***************************************************************************
void OrientedPlane3Factor::print(const string& s,
    const gtsam::KeyFormatter& keyFormatter) const {
  cout << "OrientedPlane3Factor Factor on " << landmarkKey_ << "\n";
  measured_p_.print("Measured Plane");
  this->noiseModel_->print("  noise model: ");
}

//***************************************************************************
void OrientedPlane3DirectionPrior::print(const string& s,
    const gtsam::KeyFormatter& keyFormatter) const {
  cout << "Prior Factor on " << landmarkKey_ << "\n";
  measured_p_.print("Measured Plane");
  this->noiseModel_->print("  noise model: ");
}

//***************************************************************************
bool OrientedPlane3DirectionPrior::equals(const gtsam::NonlinearFactor& expected,
    double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != NULL && Base::equals(*e, tol)
      && this->measured_p_.equals(e->measured_p_, tol);
}

//***************************************************************************

gtsam::Vector OrientedPlane3DirectionPrior::evaluateError(const OrientedPlane3& plane,
    boost::optional<gtsam::Matrix&> H) const {

  if (H) {
    gtsam::Matrix H_p;
    gtsam::Unit3 n_hat_p = measured_p_.normal();
    gtsam::Unit3 n_hat_q = plane.normal();
    gtsam::Vector e = n_hat_p.error(n_hat_q, H_p);
    H->resize(2, 3);
    H->block<2, 2>(0, 0) << H_p;
    H->block<2, 1>(0, 2) << gtsam::zeros(2, 1);
    return e;
  } else {
    gtsam::Unit3 n_hat_p = measured_p_.normal();
    gtsam::Unit3 n_hat_q = plane.normal();
    gtsam::Vector e = n_hat_p.error(n_hat_q);
    return e;
  }

}
}

