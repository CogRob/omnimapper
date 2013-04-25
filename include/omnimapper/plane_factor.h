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
#include <gtsam/nonlinear/Ordering.h>
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
