/**
 * @file    PlaneFactor.cpp
 * @brief   A non-linear factor for planar patches
 * @author  Alex Trevor
 * @author  John Rogers
 */

//#include <omnimapper/omni_config.h>
#include <omnimapper/plane_factor.h>
using namespace std;

namespace gtsam {

template <typename PointT>
void PlaneFactor<PointT>::print(const std::string& s) const {
  std::cout << s << ": Plane factor(" << (std::string)poseSymbol_ << ","
            << (std::string)landmarkSymbol_ << ")\n";
  gtsam::print(measured_, std::string("measured"));
  this->noiseModel_->print("  noise model");
}

template <typename PointT>
Vector PlaneFactor<PointT>::evaluateError(const Pose3& pose,
                                          const Plane<PointT>& plane,
                                          boost::optional<Matrix&> H1,
                                          boost::optional<Matrix&> H2) const {
  if (H1) *H1 = plane.GetDh1(pose);
  if (H2) *H2 = plane.GetDh2(pose);

  Vector xo = plane.GetXo(pose);
  return plane.Geth(xo, measured_);
}

/*  Vector PlaneFactor::error_vector(const OmniConfig& config) const {
  return Geth(config);
}

Vector PlaneFactor::
unwhitenedError(const OmniConfig& config,
                boost::optional<std::vector<Matrix>&>mats) const {
  return error_vector(config);
}

Vector  PlaneFactor::Geth(const OmniConfig& config)const {
  Pose3 X = config[poseSymbol_];
  Plane p_map = config[landmarkSymbol_];

  Vector xo = p_map.GetXo(X);
  Vector h = p_map.Geth(xo,measured_);
  return h;
}

GaussianFactor::shared_ptr PlaneFactor::
linearize(const OmniConfig& config, const Ordering& ordering) const {
  Vector b = Geth(config);

  Plane p = config[landmarkSymbol_];
  Pose3 xr = config[poseSymbol_];

  Matrix Dh1 = p.GetDh1(xr);
  Matrix Dh2 = p.GetDh2(xr);


  this->noiseModel_->WhitenSystem(Dh1, Dh2, b);

  const Index var1 = ordering[poseSymbol_];
  const Index var2 = ordering[landmarkSymbol_];

  if(var1 < var2)
    return GaussianFactor::shared_ptr(new JacobianFactor(var1, Dh1, var2, Dh2,
-b, noiseModel::Unit::Create(b.size()))); else return
GaussianFactor::shared_ptr(new JacobianFactor(var2, Dh2, var1, Dh1, -b,
noiseModel::Unit::Create(b.size())));

}

IndexFactor::shared_ptr PlaneFactor::
symbolic(const Ordering& ordering) const {
  const Index var1 = ordering[poseSymbol_], var2 = ordering[landmarkSymbol_];
  if(var1 < var2)
    return IndexFactor::shared_ptr(new IndexFactor(var1, var2));
  else
    return IndexFactor::shared_ptr(new IndexFactor(var2, var1));
}
*/
}  // namespace gtsam

template class gtsam::PlaneFactor<pcl::PointXYZ>;
template class gtsam::PlaneFactor<pcl::PointXYZRGBA>;
