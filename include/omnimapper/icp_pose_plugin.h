#pragma once

#include <omnimapper/pose_plugin.h>
#include <pcl/io/pcd_grabber.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose3.h>

namespace omnimapper
{
  /** \brief ICPPoseMeasurementPlugin adds sequential pose constraints based on scan matching to the SLAM problem.
   *
   * \author Alex Trevor
   */
  template <typename PointT>
  class ICPPoseMeasurementPlugin //: public omnimapper::PosePlugin
  {
    typedef typename pcl::PointCloud<PointT> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    typedef typename boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose3> > BetweenPose3Ptr;

    public:

      ICPPoseMeasurementPlugin (omnimapper::OmniMapperBase* mapper, pcl::Grabber& grabber); 
      ~ICPPoseMeasurementPlugin ();
      //bool addInitialPose ();
      void spin ();
      bool spinOnce ();
      bool registerClouds (CloudConstPtr& cloud1, CloudConstPtr& cloud2, CloudPtr& aligned_cloud2, Eigen::Matrix4f& tform, double& score);
      void cloudCallback (const CloudConstPtr& cloud);
      bool ready ();
      CloudConstPtr getCloudPtr (gtsam::Symbol sym);
      void setMaxCorrespondenceDistance (float max_correspondence_distance) { icp_max_correspondence_distance_ = max_correspondence_distance; }
      void setShouldDownsample (bool should_downsample) { downsample_ = should_downsample; }
      void setLeafSize (float leaf_size) { leaf_size_ = leaf_size; }
      void setUseGICP (bool use_gicp) { use_gicp_ = use_gicp; }
      void pause (bool pause);
      

    protected:
      OmniMapperBase* mapper_;
      bool initialized_;
      std::map<gtsam::Symbol, CloudConstPtr> clouds_;
      pcl::Grabber& grabber_;
      CloudConstPtr current_cloud_;
      boost::mutex current_cloud_mutex_;
      bool have_new_cloud_;
      bool first_;
      bool downsample_;
      float leaf_size_;
      bool debug_;
      bool overwrite_timestamps_;
      gtsam::Symbol previous_sym_;
      float icp_max_correspondence_distance_;
      bool use_gicp_;
      bool paused_;
  };
}
