#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <omnimapper/omnimapper_base.h>
#include <omnimapper/organized_feature_extraction.h>
#include <omnimapper/get_transform_functor.h>

#include <cloudcv/temporal_segmentation.h>
#include <cloudcv/feature_matches.h>

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
					void(std::vector<CloudPtr>, std::map<int, int>,
							std::map<int, std::map<int, int> >, int,
							omnimapper::Time)>& fn);

      CloudPtrVector getObservations (gtsam::Symbol sym);

      void setSensorToBaseFunctor (omnimapper::GetTransformFunctorPtr get_transform) { get_sensor_to_base_ = get_transform; }

      // The object descriptors stored in the database is loaded
      void loadRepresentations();


    protected:
      bool cloud_cv_flag_;
      OmniMapperBase* mapper_;
      GetTransformFunctorPtr get_sensor_to_base_;
      CloudPtrVector empty_;
      std::map<gtsam::Symbol, CloudPtrVector> observations_;
      boost::function<void(std::vector<CloudPtr>, std::map<int, int>,
				std::map<int, std::map<int, int> >, int,
				omnimapper::Time)> cloud_cv_callback_;

      TemporalSegmentation<PointT> temporal_segmentation_;
      boost::shared_ptr<FeatureMatches<pcl::SHOT1344> > correspondence_estimator;
      std::vector<pcl::PointCloud<pcl::SHOT1344> > feature_files;
      std::vector<pcl::PointCloud<pcl::PointXYZI> > keypoint_files;

      int max_object_size;
  };
}
