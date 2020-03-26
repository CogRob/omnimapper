#include <omnimapper/object.h>

namespace omnimapper {

template <typename PointT>
Object<PointT>::Object()
    : clusters_(), optimal_cloud_(new Cloud()), landmark_(false) {}

template <typename PointT>
Object<PointT>::Object(const Object<PointT>& object)
    : sym_(object.sym),
      clusters_(object.clusters_),
      indices_(object.indices_),
      factor_flag_(object.factor_flag),
      optimal_cloud_(object.optimal_cloud_),
      landmark_(object.landmark) {}
template <typename PointT>
void Object<PointT>::AddObservation(
    gtsam::Symbol sym, CloudPtr cluster,
    boost::optional<pcl::PointIndices> indices) {
  object_mutex_.lock();  // lock the object
  clusters_.insert(std::pair<gtsam::Symbol, CloudPtr>(sym, cluster));
  indices_.insert(std::pair<gtsam::Symbol, pcl::PointIndices>(sym, *indices));
  object_mutex_.unlock();  // unlock the object
}

template <typename PointT>
typename Object<PointT>::Cloud Object<PointT>::OptimalCloud() {
  object_mutex_.lock();  // lock the object
  Cloud cloud = *optimal_cloud_;
  object_mutex_.unlock();  // unlock the object
  return cloud;
}

}  // namespace omnimapper

template class omnimapper::Object<pcl::PointXYZRGBA>;
