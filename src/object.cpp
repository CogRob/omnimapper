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

	clusters_.insert(std::pair<gtsam::Symbol, CloudPtr>(sym, cluster));
	indices_.insert(std::pair<gtsam::Symbol, pcl::PointIndices >(sym, *indices));

}

}

template class omnimapper::Object<pcl::PointXYZRGBA>;
