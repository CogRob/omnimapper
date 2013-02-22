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
#include <gtsam/nonlinear/Key.h>
#include <gtsam/geometry/Pose3.h>
//#include <mapping/sam_config.h>
#include <gtsam/nonlinear/Ordering.h>
#include <gtsam/linear/SharedGaussian.h>
//#include <pcl/transforms.h>
//#include <gtpointcloud/pointcloud_helpers.h>

#include <mapping/plane.h>

namespace gtsam {


/**
 * A planar feature.
 */
class PlaneFactor : public NoiseModelFactor
//class PlaneFactor : public NonlinearFactor<OmniConfig>
{
 private:

    //typedef NoiseModelFactor<SAMConfig> ConvenientFactor;
  //typedef NonlinearFactor<OmniConfig> ConvenientFactor;
 protected:
    //PoseKey poseSymbol_;
    //PlaneKey landmarkSymbol_;
    Key poseSymbol_;
    Key landmarkSymbol_;
    
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
    //PlaneFactor(const Vector& z, const SharedGaussian& noiseModel, 
	  //     const PoseKey& poseNumber, 
	  //     const PlaneKey& landmarkNumber):
    PlaneFactor(const Vector& z, const SharedGaussian& noiseModel, 
                const Key& poseNumber, 
                const Key& landmarkNumber):
      NoiseModelFactor(noiseModel, poseNumber, landmarkNumber),
      poseSymbol_(poseNumber),
      landmarkSymbol_(landmarkNumber),
      measured_(z) {
      //keys_.push_back(poseSymbol_);
      //      keys_.push_back(landmarkSymbol_);
  }
  Key getPoseSymbol() const { return poseSymbol_;}
  Key getLandmarkSymbol() const{ return landmarkSymbol_;}
  void setLandmarkSymbol(const Key& newLandmarkSymbol) {
    landmarkSymbol_ = newLandmarkSymbol;
  }
  /**
   * print
   * @param s optional string naming the factor
   */
  void print(const std::string& s="PlaneFactor") const;
  
  /**
   * predict the measurement
   */
  //Vector predict(const OmniConfig&) const;
  //Vector predict(const SAMConfig&) const;
    Vector predict (const gtsam::Values&) const;
    

  /**
   * calculate the error of the factor
   */
  //Vector error_vector(const OmniConfig& config) const;
  //Vector error_vector(const SAMConfig& config) const;
    Vector error_vector(const gtsam::Values& config) const;
  //Vector unwhitenedError(const OmniConfig& config, 
  //			 boost::optional<std::vector<Matrix>&> mats= boost::none) const; 
//  Vector unwhitenedError(const SAMConfig& config, 
//  			 boost::optional<std::vector<Matrix>&> mats= boost::none) const; 

    Vector unwhitenedError(const gtsam::Values& config, 
                           boost::optional<std::vector<Matrix>&> mats= boost::none) const; 
    
  /**
   * linerarization
   */

  ///Plane stuff

  // h: the predicted measurement 
  //Vector Geth(const OmniConfig& config)const;
  //Vector Geth(const SAMConfig& config)const;
  Vector Geth(const gtsam::Values& config)const;
    

  //GaussianFactor::shared_ptr linearize(const OmniConfig&config, const Ordering& ordering) const;
  //GaussianFactor::shared_ptr linearize(const SAMConfig&config, const Ordering& ordering) const;
    GaussianFactor::shared_ptr linearize(const gtsam::Values&config, const Ordering& ordering) const;
  IndexFactor::shared_ptr symbolic(const Ordering& ordering) const; 
 };

}
