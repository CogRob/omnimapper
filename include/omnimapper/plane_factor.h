/**
 * @file    PlaneFactor.h
 * @brief   A 3D Plane factor
 * @author  Alex Trevor
 * @author  John Rogers 
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Symbol.h>
//#include <gtsam/nonlinear/Key.h>
//#include <omnimapper/omni_config.h>
//#include <gtsam/nonlinear/Ordering.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/geometry/Pose3.h>
#include <omnimapper/plane.h>
//#include <gtsam/linear/SharedGaussian.h>
//#include <pcl_ros/transforms.h>
//#include <gtpointcloud/pointcloud_helpers.h>
#include <pcl/point_types.h>


namespace gtsam {


/**
 * A planar feature.
 */
  template <typename PointT>
  class PlaneFactor : public NoiseModelFactor2<Pose3, Plane<PointT> >
{
 private:
    typedef PlaneFactor<PointT> This;
  typedef NoiseModelFactor2<Pose3, Plane<PointT> > Base;
 protected:
  Symbol poseSymbol_;
  Symbol landmarkSymbol_;
  Vector measured_;
 public:

  typedef boost::shared_ptr<PlaneFactor> shared_ptr; // shorthand for a smart pointer to a factor

  /**
   * Constructor
   * @param z is the 2 dimensional location of point in image (the measurement)
   * @param noiseModel is the Gaussian noise model for this measurement
   * @param poseNumber is the symbol for this robot pose
   * @param landmarkNumber is the index of the landmark
   * @param K the constant calibration
   */
  PlaneFactor() {}
  PlaneFactor(const Vector& z, const SharedGaussian& noiseModel, 
	       const Symbol& pose, 
	       const Symbol& landmark):
  Base(noiseModel, 
       pose, landmark),
    poseSymbol_(pose),
    landmarkSymbol_(landmark),
    measured_(z) {
  }
    // PlaneFactor<PointT> (const PlaneFactor<PointT>& p) 
    // :  PlaneFactor<PointT> (p.measured_, p.noiseModel_, p.poseSymbol_, p.landmarkSymbol_)
    // {}
  //   PlaneFactor (const PlaneFactor& p)
  // : measured_ (p.measured_),
  // noiseModel_ (p.noiseModel_),
  // poseSymbol_ (p.poseSymbol_),
  // landmarkSymbol_ (p.landmarkSymbol_) 
  //   {}
  

    
  Symbol getPoseSymbol() const { return poseSymbol_;}
  Symbol getLandmarkSymbol() const{ return landmarkSymbol_;}
  void setLandmarkSymbol(const Symbol& newLandmarkSymbol) {
    landmarkSymbol_ = newLandmarkSymbol;
  }
  /**
   * print
   * @param s optional string naming the factor
   */
  void print(const std::string& s="PlaneFactor") const;

    /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                                                              gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

  virtual Vector evaluateError(const Pose3& pose, const Plane<PointT>& plane, 
			       boost::optional<Matrix&> H1 = boost::none,
			       boost::optional<Matrix&> H2 = boost::none)const;
  private:
    /** Serialization function */

    friend class boost::serialization::access;

    template<class Archive>
      void serialize(Archive & ar, const unsigned int version)
      {
	printf("Seralizing a plane factor!\n");
	ar & boost::serialization::make_nvp("PlaneFactor",
					    boost::serialization::base_object<Base>(*this));
	printf("A\n");
	ar & boost::serialization::make_nvp("PoseKey", 
					    poseSymbol_);
	printf("B\n");
	ar & boost::serialization::make_nvp("PlaneKey", 
					    landmarkSymbol_);
	printf("C\n");
	ar & boost::serialization::make_nvp("measured",
					    measured_);
	printf("Done\n");
      }

 };

}
