#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <omnimapper/omnimapper_base.h>
#include <omnimapper/organized_feature_extraction.h>
#include <omnimapper/get_transform_functor.h>
#include <omnimapper/object.h>
#include <omnimapper/landmark_factor.h>
#include <omnimapper/object_segment_propagation.h>
#include <omnimapper/object_recognition.h>
#include <omnimapper/object_discovery.h>

#include <cpu_tsdf/tsdf_volume_octree.h>
#include <cpu_tsdf/marching_cubes_tsdf_octree.h>

#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

namespace omnimapper
{
  /** \brief ObjectPlugin keeps track of objects, both recognized and unrecognized, and optionally localizes from objects flagged as stationary.
   *  \author Alex Trevor
   */
  template<typename PointT>
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

      void clusterCloudCallback (std::vector<CloudPtr> clusters,
          omnimapper::Time t, boost::optional<std::vector<pcl::PointIndices> >);

      void setObjectCallback (
          boost::function<
              void (std::map<gtsam::Symbol, gtsam::Object<PointT> >)>& fn);

      CloudPtrVector getObservations (gtsam::Symbol sym);

      void setSensorToBaseFunctor (
          omnimapper::GetTransformFunctorPtr get_transform)
      {
        get_sensor_to_base_ = get_transform;
      }

      // The object descriptors stored in the database is loaded
      void loadRepresentations ();
      void recognizeObject (gtsam::Object<PointT>& object, int id);
      void objectRecognitionLoop ();
      void objectDiscoveryLoop ();
      float computeIntersection (Eigen::Vector4f minA, Eigen::Vector4f maxA,
          Eigen::Vector4f minB, Eigen::Vector4f maxB);

      gtsam::Symbol popFromQueue ();
      void pushIntoQueue (gtsam::Symbol sym);
      void generateObjectModel (gtsam::Object<PointT> object,
          Eigen::Vector4f obj_centroid, int id);
      void reconstructSurface (CloudPtr cloud, int id);

      void update (boost::shared_ptr<gtsam::Values>& vis_values,
          boost::shared_ptr<gtsam::NonlinearFactorGraph>& vis_graph);
      void recreateObjectModels ();

      void setObjectDatabaseLocation(std::string object_database_location){
        object_database_location_ = object_database_location;
      }


    protected:
      bool cloud_cv_flag_, recompute_flag_;
      OmniMapperBase* mapper_;
      GetTransformFunctorPtr get_sensor_to_base_;
      CloudPtrVector empty_;
      std::map<gtsam::Symbol, CloudPtrVector> observations_;
      std::map<gtsam::Symbol, std::vector<pcl::PointIndices> > observation_indices_;

      boost::function<
          void (std::map<gtsam::Symbol, gtsam::Object<PointT> >)> cloud_cv_callback_;

      boost::shared_ptr<SegmentPropagation<PointT> > segment_propagation_;
      boost::shared_ptr<ObjectRecognition<pcl::SHOT1344> > object_recognition_;
      ObjectDiscovery<pcl::PointXYZRGBA> object_discovery_;

      std::vector<pcl::PointCloud<pcl::SHOT1344> > feature_files;
      std::vector<pcl::PointCloud<pcl::PointXYZI> > keypoint_files;
      std::map<gtsam::Symbol, gtsam::Object<PointT> > object_map;
      std::map<gtsam::Symbol, gtsam::Symbol> omnimapper_graph;

      std::map<gtsam::Symbol, int> training_map;
      std::queue<gtsam::Symbol> train_queue;
      int max_object_size, max_current_size;
      cpu_tsdf::TSDFVolumeOctree::Ptr tsdf;
      boost::mutex recog_mutex;

      std::string object_database_location_;

  };
}
