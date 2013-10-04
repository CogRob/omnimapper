#include <omnimapper/object.h>

namespace gtsam {

template<typename PointT>
Object<PointT>::Object() :
		clusters_(), landmark(false), optimal_cloud_(new Cloud()) {

}

template<typename PointT>
void Object<PointT>::addObservation(Symbol sym, CloudPtr cluster, boost::optional<pcl::PointIndices> indices) {

	clusters_.insert(std::pair<Symbol, CloudPtr>(sym, cluster));
	indices_.insert(std::pair<Symbol, pcl::PointIndices >(sym, *indices));

}

}

template class gtsam::Object<pcl::PointXYZRGBA>;
