/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  LandmarkTransformFactor.h
 *  @author Radu B. Rusu
 **/
#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Testable.h>

namespace gtsam {

	/**
	 * A class for a rigid pose estimate using landmarks.
	 * @addtogroup SLAM
	 */
  template<typename POSE, typename LANDMARK>
  class LandmarkTransformFactor: public NoiseModelFactor2<POSE, LANDMARK> {

  protected:
    LANDMARK measured_;
  public:
    typedef NoiseModelFactor2<POSE, LANDMARK> Base;
    typedef LandmarkTransformFactor<POSE, LANDMARK> This;

    LandmarkTransformFactor (Key poseKey, 
                             Key pointKey,
                             const LANDMARK& measured, 
                             const SharedNoiseModel& model)
      : Base (model, poseKey, pointKey), measured_ (measured)
    {
    }

    virtual ~LandmarkTransformFactor () {}

    virtual gtsam::NonlinearFactor::shared_ptr clone () const {
      return boost::static_pointer_cast<gtsam::NonlinearFactor> (
        gtsam::NonlinearFactor::shared_ptr (new This (*this)));
    }


    virtual Vector
    evaluateError (const POSE &pose, const LANDMARK &point,
                   boost::optional<Matrix&> H1 = boost::none,
                   boost::optional<Matrix&> H2 = boost::none) const
    {
      // Transform the point and calculate the error. Jacobians are computed internally.
      return (measured_.localCoordinates (pose.transform_to (point, H1, H2)));
    }

    const LANDMARK&
    measured () const
    {
      return (measured_);
    }

    virtual bool 
    equals (const NonlinearFactor& expected, double tol = 1e-9) const 
    {
      const This *e = dynamic_cast<const This*> (&expected);
      return (e != NULL && Base::equals (*e, tol) && this->measured_.equals (e->measured_, tol));
    }
//
//    void
//    print (const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const
//    {
//      std::cout << s << "LandmarkTransformFactor, range = " << measured_ << std::endl;
//      Base::print ("", keyFormatter);
//    }

	private:

		/** Serialization function */
		friend class boost::serialization::access;
		//template<class ARCHIVE>
		//void serialize(ARCHIVE & ar, const unsigned int version) {
		//	ar & boost::serialization::make_nvp("NoiseModelFactor1",
		//			boost::serialization::base_object<Base>(*this));
		//	ar & BOOST_SERIALIZATION_NVP(prior_);
		//}
	};

} /// namespace gtsam
