#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <omnimapper/omnimapper_base.h>
#include <omnimapper/organized_feature_extraction.h>

namespace omnimapper
{
  /** \brief ObjectPlugin keeps track of objects, both recognized and unrecognized, and optionally localizes from objects flagged as stationary.
   *  \author Alex Trevor
   */
  template <typename PointT>
  class ObjectPlugin
  {

    typedef typename pcl::PointCloud<PointT> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;
    //typedef typename std::vector<CloudPtr, Eigen::aligned_allocator<CloudPtr> > CloudPtrVector;
    typedef typename std::vector<CloudPtr> CloudPtrVector;

    public:
      ObjectPlugin (omnimapper::OmniMapperBase* mapper);
      ~ObjectPlugin ();
      
      void clusterCloudCallback (std::vector<CloudPtr> clusters, omnimapper::Time t);

      void setObjectCallback(
			boost::function<
					void(gtsam::Symbol, boost::optional<gtsam::Pose3>,
							std::vector<CloudPtr>, omnimapper::Time t)>& fn);

      CloudPtrVector getObservations (gtsam::Symbol sym);

    protected:
      bool cloud_cv_flag_;
      OmniMapperBase* mapper_;
      CloudPtrVector empty_;
      std::map<gtsam::Symbol, CloudPtrVector> observations_;
      boost::function<void(gtsam::Symbol, boost::optional<gtsam::Pose3>, std::vector<CloudPtr>, omnimapper::Time t)> cloud_cv_callback_;
  };
}
