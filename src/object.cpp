#include <omnimapper/object.h>

namespace omnimapper {

template<typename PointT>
Object<PointT>::Object() :
		clusters_(), landmark(false), optimal_cloud_(new Cloud()){

}


template<typename PointT>
Object<PointT>::Object(const Object<PointT>& object):
      clusters_ (object.clusters_), landmark (object.landmark), optimal_cloud_ (
          object.optimal_cloud_), factor_flag(object.factor_flag),
          indices_(object.indices_),sym(object.sym)
{


}
template<typename PointT>
void Object<PointT>::addObservation(gtsam::Symbol sym, CloudPtr cluster, boost::optional<pcl::PointIndices> indices) {

  object_mutex_.lock(); // lock the object
	clusters_.insert(std::pair<gtsam::Symbol, CloudPtr>(sym, cluster));
	indices_.insert(std::pair<gtsam::Symbol, pcl::PointIndices >(sym, *indices));
	object_mutex_.unlock(); // unlock the object

}


template<typename PointT>
typename Object<PointT>::Cloud Object<PointT>::optimalCloud()
{
  object_mutex_.lock(); // lock the object
  Cloud cloud = *optimal_cloud_;
  object_mutex_.unlock(); // unlock the object
  return cloud;
}


}



template class omnimapper::Object<pcl::PointXYZRGBA>;
