/**
 * @file    PlaneFactor.cpp
 * @brief   A non-linear factor for planar patches
 * @author  Alex Trevor
 * @author  John Rogers
 */

//#include <mapping/sam_config.h>
#include <mapping/plane_factor.h>

using namespace std;
namespace gtsam{
  


  void PlaneFactor::print(const std::string& s) const {
    std::cout <<s <<": Plane factor(" << "TODO: FIX PRINT FUNCTION\n";
      //<< (std::string) poseSymbol_ << ","
      //      << (std::string) landmarkSymbol_ <<")\n";
    gtsam::print(measured_,std::string("measured"));
    this->noiseModel_->print("  noise model");
  }

  /**
   * calculate the error of the factor
   */
  //Vector PlaneFactor::error_vector(const SAMConfig& config) const {
  Vector PlaneFactor::error_vector(const Values& config) const {
    return Geth(config);
  }

  Vector PlaneFactor::
  //unwhitenedError(const SAMConfig& config, 
  //		  boost::optional<std::vector<Matrix>&>mats) const {
  unwhitenedError(const Values& config, 
  		  boost::optional<std::vector<Matrix>&>mats) const {
    return error_vector(config);
  }

  //Vector  PlaneFactor::Geth(const SAMConfig& config)const {
  Vector  PlaneFactor::Geth(const Values& config)const {
    //Pose3 X = config[poseSymbol_];
    Pose3 X = config.at<Pose3>(poseSymbol_);
    //Plane p_map = config[landmarkSymbol_];
    Plane p_map = config.at<Plane>(landmarkSymbol_);

    Vector xo = p_map.GetXo(X);
    Vector h = p_map.Geth(xo,measured_);

    //TODO: remove this
      /*
    tf::Transform pose2map = btTransform(tf::createQuaternionFromYaw(X.theta()),btVector3(X.x(), X.y(), 0.0));

    Eigen::Vector4f pred_meas;
    pred_meas[0] = cos(X.theta())*p_map.a()+sin(X.theta())*p_map.b();
    pred_meas[1] = -sin(X.theta())*p_map.a()+cos(X.theta())*p_map.b();
    pred_meas[2] = p_map.c();
    pred_meas[3] = p_map.a()*X.x()+p_map.b()*X.y()+p_map.d();
    
    h[0] = pred_meas[0] - measured_[0];
    h[1] = pred_meas[1] - measured_[1];
    h[2] = pred_meas[2] - measured_[2];
    h[3] = pred_meas[3] - measured_[3];
      */

    return h;
  }
  
  //GaussianFactor::shared_ptr PlaneFactor::
  //linearize(const SAMConfig& config, const Ordering& ordering) const {
  GaussianFactor::shared_ptr PlaneFactor::
  linearize(const Values& config, const Ordering& ordering) const {
    Vector b = Geth(config);
    
    //Plane p = config[landmarkSymbol_];
    //Pose3 xr = config[poseSymbol_];
    Plane p = config.at<Plane>(landmarkSymbol_);
    Pose3 xr = config.at<Pose3>(poseSymbol_);

    Matrix Dh1 = p.GetDh1(xr);
    Matrix Dh2 = p.GetDh2(xr);


    this->noiseModel_->WhitenSystem(Dh1, Dh2, b);
    //this->noiseModel_->WhitenInPlace(Dh1);
    //this->noiseModel_->WhitenInPlace(Dh2);
    //this->noiseModel_->whitenInPlace(b);

    const Index var1 = ordering[poseSymbol_];
    const Index var2 = ordering[landmarkSymbol_];
    
    if(var1 < var2)
      return GaussianFactor::shared_ptr(new JacobianFactor(var1, Dh1, var2, Dh2, -b, noiseModel::Unit::Create(b.size())));
    else
      return GaussianFactor::shared_ptr(new JacobianFactor(var2, Dh2, var1, Dh1, -b, noiseModel::Unit::Create(b.size())));

  }

  IndexFactor::shared_ptr PlaneFactor::
  symbolic(const Ordering& ordering) const {
    const Index var1 = ordering[poseSymbol_], var2 = ordering[landmarkSymbol_];
    if(var1 < var2)
      return IndexFactor::shared_ptr(new IndexFactor(var1, var2));
    else
      return IndexFactor::shared_ptr(new IndexFactor(var2, var1));
  }
    
}
