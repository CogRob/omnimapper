#pragma once

#include <cpu_tsdf/marching_cubes_tsdf_octree.h>
#include <cpu_tsdf/tsdf_volume_octree.h>
#include <omnimapper/get_transform_functor.h>
#include <omnimapper/landmark_factor.h>
#include <omnimapper/object.h>
#include <omnimapper/object_discovery.h>
#include <omnimapper/object_recognition.h>
#include <omnimapper/object_segment_propagation.h>
#include <omnimapper/omnimapper_base.h>
#include <omnimapper/organized_feature_extraction.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

namespace omnimapper {
/** \brief ObjectPlugin keeps track of objects, both recognized and
 * unrecognized, and optionally localizes from objects flagged as stationary.
 *  \author Alex Trevor
 *  \author Siddharth Choudhary
 */
template <typename PointT>
class ObjectPlugin {
  typedef typename pcl::PointCloud<PointT> Cloud;
  typedef typename Cloud::Ptr CloudPtr;
  typedef typename Cloud::ConstPtr CloudConstPtr;
  typedef typename std::vector<CloudPtr> CloudPtrVector;

 public:
  /** \brief ObjectPlugin constructor */
  ObjectPlugin(omnimapper::OmniMapperBase* mapper);

  /** \brief Destructor               */
  ~ObjectPlugin();

  /** \brief clusterCloudCallback receives non planar segments
   *   from organized feature extraction and filters out object
   *   hypothesis from those segments, aggregates the object hypothesis
   *   to form a single object cloud and stores the feature description
   */
  void ClusterCloudCallback(std::vector<CloudPtr> clusters, omnimapper::Time t,
                            boost::optional<std::vector<pcl::PointIndices> >);

  /** \brief setObjectCallback sets the visualization call back
   *  to connect object plugin to the visualization module
   */
  void SetObjectCallback(
      boost::function<void(std::map<gtsam::Symbol, Object<PointT> >,
                           gtsam::Point3, gtsam::Point3)>& fn);

  /** \brief getObservations is used by visualization module
   * to visualize filtered out object observations
   */
  CloudPtrVector GetObservations(gtsam::Symbol sym);

  /** \brief setSensorToBaseFunctor provides the transformation from
   * rgbd frame to base frame.
   */
  void SetSensorToBaseFunctor(
      omnimapper::GetTransformFunctorPtr get_transform) {
    get_sensor_to_base_ = get_transform;
  }

  /** \brief The object descriptors stored in the database is loaded */
  void LoadDatabase();

  /** \brief recognizeObject matches the current object representation
   *   against the saved representations
   */
  void RecognizeObject(Object<PointT>& object);

  /** \brief loop to recognize objects that are pushed in the queue */
  void ObjectRecognitionLoop();

  /** \brief take an object out of the recognition queue */
  gtsam::Symbol PopFromQueue();

  /** \brief push an object in the recognition queue */
  void PushIntoQueue(gtsam::Symbol sym);

  /** \brief takes in all objects and undersegmented object parts and merge them
   * using connected component to form full objects */
  void ObjectDiscoveryLoop();

  float ComputeViewIntersection(gtsam::Point3 view_direction,
                                gtsam::Point3 view_center,
                                Eigen::Vector4f obj_center);

  float ComputeIntersection(Eigen::Vector4f minA, Eigen::Vector4f maxA,
                            Eigen::Vector4f minB, Eigen::Vector4f maxB);

  /** \brief computeTSDF estimates the object TSDF */
  void ComputeTSDF(Object<PointT> object, Eigen::Vector4f obj_centroid);

  /** \brief reconstruct surface */
  void ReconstructSurface(CloudPtr cloud, int id);

  void Update(boost::shared_ptr<gtsam::Values>& vis_values,
              boost::shared_ptr<gtsam::NonlinearFactorGraph>& vis_graph);

  /** \brief Uses the optimal poses to reconstruct objects */
  void ComputeOptimalObjectModel();

  /** \brief location where the pcd files and descriptors are saved */
  void SetAndLoadObjectDatabaseLocation(std::string object_database_location);

  /** \brief use objects for loop closures or not */
  void UseObjectLoopClosures(bool do_loop_closures) {
    do_loop_closures_ = do_loop_closures;
  }

  /** \brief use objects as landmarks  or not */
  void UseObjectLandmarks(bool use_object_landmarks) {
    use_object_landmarks_ = use_object_landmarks;
  }

  /** \brief do object model save or not */
  void SaveObjectModels(bool save_object_models) {
    save_object_models_ = save_object_models;
  }

  /** \brief set minimum cluster height */
  void SetMinimumClusterHeight(double min_cluster_height) {
    min_cluster_height_ = min_cluster_height;
  }

 protected:
  OmniMapperBase* mapper_;
  GetTransformFunctorPtr get_sensor_to_base_;
  std::map<gtsam::Symbol, CloudPtrVector> observations_;
  CloudPtrVector empty_;
  std::map<gtsam::Symbol, std::vector<pcl::PointIndices> > observation_indices_;

  int max_object_size_, max_current_size_;
  cpu_tsdf::TSDFVolumeOctree::Ptr tsdf_;

  bool vis_flag_;              // flag to check if visualization callback is set
  bool debug_, verbose_;       // flags to control the couts
  bool do_loop_closures_;      // flag to control object object loop closures
  bool save_object_models_;    // flag to control object model r/w
  bool use_object_landmarks_;  // flag to control whether objects are used as
                               // landmarks or not

  double min_cluster_height_;  // height above the floor for each cluster

  /** vis_callback_ calls object visualization function in
   * OmnimapperVisualizerRviz */
  boost::function<void(std::map<gtsam::Symbol, Object<PointT> >,
                       gtsam::Point3 center, gtsam::Point3 direction)>
      vis_callback_;

  boost::shared_ptr<SegmentPropagation<PointT> > segment_propagation_;
  boost::shared_ptr<ObjectRecognition<pcl::SHOT1344> > object_recognition_;
  boost::shared_ptr<ObjectDiscovery<pcl::PointXYZRGBA> > object_discovery_;

  std::vector<pcl::PointCloud<pcl::SHOT1344> > feature_files_;
  std::vector<pcl::PointCloud<pcl::PointXYZI> > keypoint_files_;
  std::map<gtsam::Symbol, Object<PointT> > object_map_;
  std::map<gtsam::Symbol, gtsam::Symbol> omnimapper_graph_;

  std::map<gtsam::Symbol, int> training_map_;
  std::queue<gtsam::Symbol> train_queue_;

  /* mutexes */
  boost::mutex recog_mutex_;         // mutex for recognition queue
  boost::mutex map_building_mutex_;  // mutex for optimal map creation

  std::string object_database_location_;
};
}  // namespace omnimapper
