#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <omnimapper/omnimapper_base.h>
#include <organized_segmentation_tools/organized_segmentation_tbb.h>
#include <omnimapper/get_transform_functor.h>
#include <omnimapper/object.h>
#include <omnimapper/landmark_factor.h>
//#include <omnimapper/object_segment_propagation.h>
//#include <omnimapper/object_recognition.h>
//#include <omnimapper/object_discovery.h>

//#include <cpu_tsdf/tsdf_volume_octree.h>
//#include <cpu_tsdf/marching_cubes_tsdf_octree.h>

#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/gicp.h>

namespace omnimapper
{
  /** \brief ObjectPlugin keeps track of objects, both recognized and unrecognized, and optionally localizes from objects flagged as stationary.
   *  \author Alex Trevor
   *  \author Siddharth Choudhary
   */
  template<typename PointT>
  class ObjectPlugin
  {

      typedef typename pcl::PointCloud<PointT> Cloud;
      typedef typename Cloud::Ptr CloudPtr;
      typedef typename Cloud::ConstPtr CloudConstPtr;
      typedef typename std::vector<CloudPtr> CloudPtrVector;
      typedef typename boost::shared_ptr < Object<PointT> > ObjectPtr;


    public:

      /** \brief ObjectPlugin constructor */
      ObjectPlugin (omnimapper::OmniMapperBase* mapper);

      /** \brief Destructor               */
      ~ObjectPlugin ();

      /** \brief clusterCloudCallback receives non planar segments
       *   from organized feature extraction and filters out object
       *   hypothesis from those segments, aggregates the object hypothesis
       *   to form a single object cloud and stores the feature description
       *   @param clusters Vector of non planar clusters in a frame
       *   @param t The corresponding time stamp of the frame (boost::posix_time::ptime)
       *   @param indices Optional point indices corresponding to the non planar clusters
       */
      void clusterCloudCallback (std::vector<CloudPtr> clusters,
          omnimapper::Time t, boost::optional<std::vector<pcl::PointIndices> > indices);

      /** \brief processCloud receives non planar segments
       *   from organized feature extraction and filters out object
       *   hypothesis from those segments, aggregates the object hypothesis
       *   to form a single object cloud and stores the feature description
       *
       */
      void processCloud ();


      /** \brief setObjectCallback sets the visualization call back
       *  to connect object plugin to the visualization module
       */
      void setVisCallback (
          boost::function<void (std::map<gtsam::Symbol, ObjectPtr>)>& fn);

      /** \brief getObservations is used by visualization module
       * to visualize filtered out object observations
       */
      CloudPtrVector getObservations (gtsam::Symbol sym);

      /** \brief setSensorToBaseFunctor provides the transformation from
       * rgbd frame to base frame.
       */
      void setSensorToBaseFunctor (
          omnimapper::GetTransformFunctorPtr get_transform)
      {
        get_sensor_to_base_ = get_transform;
      }


      /** \brief
       *
       */
      std::pair<std::vector<gtsam::Symbol>, std::map<gtsam::Symbol, std::vector<int> > >
      dataAssociate (CloudPtrVector label, gtsam::Pose3 pose, gtsam::Symbol sym);

      /** \brief
       *
       */
      void updateObjects(
            std::vector<gtsam::Symbol> neighbor_vec,
            std::map<gtsam::Symbol, std::vector<int> > rev_neighbor_vec,
            CloudPtrVector point_cloud,
            gtsam::Symbol cloud_symbol,
            gtsam::Pose3 cloud_pose);

      /** \brief
       *
       */
      std::vector<gtsam::Symbol> linearMatch (CloudPtrVector label);

      /** \brief
       *
       */
      void refreshObjectModels();

      /** \brief
       *
       */
      void spin();

      /** \brief
       *
       */
      float computeBoundingBoxIntersection (
          std::pair<Eigen::Vector4f, Eigen::Vector4f> bounding_box_a,
          std::pair<Eigen::Vector4f, Eigen::Vector4f> bounding_box_b);



      /** \brief set minimum cluster distance */
      void setMaximumClusterDistance(double max_cluster_distance){
              max_cluster_dist_=max_cluster_distance;
      }

      /** \brief set the maximum bounding box volume threshold for each object segment*/
      void setMaximumBboxVolume (double max_bbox_volume)
      {
        max_bbox_volume_ = max_bbox_volume;
      }

      /** \brief set the maximum bounding box dimension threshold for each object segment*/
      void setMaximumBboxDim (double max_bbox_dim)
      {
        max_bbox_dim_ = max_bbox_dim;
      }

      /** \brief set the minimum curvature of each object segment */
      void setMinimumCurvature (double min_curvature)
      {
        min_curvature_ = min_curvature;
      }

      /** \brief set the minimum number of points in each object segment */
      void setMinimumPoints (double min_points)
      {
        min_points_ = min_points;
      }

      /** \brief set the minimum distance of cluster centroid from the nerest plane */
      void setMinimumClustCentroidDist (double min_clust_centroid_ptp_distance)
      {
        min_clust_centroid_ptp_dist = min_clust_centroid_ptp_distance;
      }

      /** \brief */
      void setCullThresh (double ptp_pt_cull_thresh)
      {
        ptp_pt_cull_thresh_ = ptp_pt_cull_thresh;
      }

      /** \brief set minimum cluster height */
      void setMinimumClusterHeight(double min_cluster_height){
          min_cluster_height_=min_cluster_height;
      }


      bool ready ()
      {
        boost::mutex::scoped_lock lock(object_plugin_mutex_);
        if (debug_)
          printf ("ObjectPlugin: ready: %zu\n", (!have_new_cloud_));
        return (!have_new_cloud_);
      }

      /** \brief */
      void savePCDs(bool save_pcds){save_pcds_ = save_pcds;}


      /** \brief */
      void setPCDlocation(std::string object_pcd_location){object_pcd_location_ = object_pcd_location;}



    protected:
      bool vis_flag_; ///> flag to check if visualization callback is set
      bool debug_, verbose_; ///>flags to control the couts
      bool have_new_cloud_; ///> flag to control whether there is a new cloud or not
      double min_cluster_height_; ///>height above the floor for each cluster
      double max_cluster_dist_;
      double max_bbox_volume_;
      double max_bbox_dim_;
      double min_curvature_;
      double min_points_;
      double min_clust_centroid_ptp_dist;
      double ptp_pt_cull_thresh_;
      bool filter_points_near_planes_;

      bool save_pcds_; ///> should the pcds be saved
      std::string object_pcd_location_; ///> location at which object pcds are saved

      OmniMapperBase* mapper_;
      GetTransformFunctorPtr get_sensor_to_base_   ;
      CloudPtrVector empty_;
      std::map<gtsam::Symbol, CloudPtrVector> observations_;
      std::map<gtsam::Symbol, std::vector<pcl::PointIndices> > observation_indices_;

      /** vis_callback_ calls object visualization function in OmnimapperVisualizerRviz */
      boost::function<void (std::map<gtsam::Symbol, ObjectPtr>)> vis_callback_;

      std::map<gtsam::Symbol, ObjectPtr > object_map_;

      int max_object_size, max_current_size;

      /* mutexes */
      boost::mutex object_plugin_mutex_; ///> mutex for clusterCloudCallback


      std::string object_database_location_;
      std::vector<Eigen::Vector4f> centroid_vec_;

      std::vector<CloudPtr> prev_clusters_;
      omnimapper::Time prev_time_;
      boost::optional<std::vector<pcl::PointIndices> > prev_indices_;

  };
}
