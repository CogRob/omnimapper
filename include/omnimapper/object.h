#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <omnimapper/omnimapper_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/thread/locks.hpp>

namespace omnimapper {
// For now, this is just a collection that contains a point cloud pointer and an
// optional label
template <typename PointT>
class Object {
  typedef typename pcl::PointCloud<PointT> Cloud;
  typedef typename Cloud::Ptr CloudPtr;
  typedef typename Cloud::ConstPtr CloudConstPtr;

 public:
  // constructor
  Object();

  // copy constructor
  Object(const Object& object);

  gtsam::Symbol sym_;
  // list of observations
  std::map<gtsam::Symbol, CloudPtr> clusters_;
  std::map<gtsam::Symbol, pcl::PointIndices> indices_;
  std::map<gtsam::Symbol, int> factor_flag_;
  CloudPtr optimal_cloud_;
  boost::mutex object_mutex_;

  // optional label
  std::string name_;

  // flag for use as a landmark
  bool landmark_;

  // gtsam::Symbol
  void AddObservation(gtsam::Symbol sym, CloudPtr cluster,
                      boost::optional<pcl::PointIndices> indices);
  Cloud OptimalCloud();
};

}  // namespace omnimapper
