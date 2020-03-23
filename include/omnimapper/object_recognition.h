#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/registration/transforms.h>
#include <sstream>
#include <string>
#include <vector>
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>
#include <pcl/features/3dsc.h>
#include <pcl/features/cvfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/pfh.h>
#include <pcl/features/pfhrgb.h>
#include <pcl/features/shot_omp.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <gtsam/geometry/Pose3.h>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/vector.hpp>

#define USE_KDTREE 1
struct PoseVector : std::vector<gtsam::Pose3> {
 public:
  std::vector<gtsam::Pose3> pose_vector;

 private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    // ar &  boost::serialization::base_object<std::vector<gtsam::Pose3>
    // >(*this);
    ar& pose_vector;
  }
};

template <typename FeatureType>
class ObjectRecognition {
 public:
  ObjectRecognition(
      boost::shared_ptr<pcl::Keypoint<pcl::PointXYZRGBA, pcl::PointXYZI> >
          keypoint_detector,
      typename pcl::Feature<pcl::PointXYZRGBA, FeatureType>::Ptr
          feature_extractor);

  /**
   * @brief starts the event loop for the visualizer
   */
  void run();
  void matchClouds(
      typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr source,
      typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr target);

  pcl::PointCloud<FeatureType> computeDescriptor(
      typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr source);

  void storeDescriptor(typename pcl::PointCloud<FeatureType>::Ptr features,
                       std::string filename);
  std::pair<pcl::PointCloud<FeatureType>, pcl::PointCloud<pcl::PointXYZI> >
  loadDescriptor(const std::string filename);

  void matchToStoredTemplate(
      typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr source,
      typename pcl::PointCloud<FeatureType> target_feature);

  int matchToFile(typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr source,
                  typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr target);

  std::pair<int, int> matchToFeatureFile(
      typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr source, int label,
      gtsam::Matrix covariance);

  std::pair<pcl::PointCloud<FeatureType>, pcl::PointCloud<pcl::PointXYZI> >
  computeAndStoreDescriptor(
      typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr source,
      std::string filename);

  std::pair<int, int> matchToDatabase(
      typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cluster,
      std::vector<gtsam::Pose3> pose_array, int label,
      gtsam::Matrix covariance);

  int loadDatabase(std::string dir);
  int loadMapping(std::string filename);
  int loadPCD(std::string filename);
  void saveMapping(std::string filename);
  void saveObjectStats(std::string filename);
  void savePoseArray(std::string filename, PoseVector pose_vector);
  PoseVector loadPoseArray(std::string filename);

  pcl::CorrespondencesPtr correspondences_;
  typename pcl::PointCloud<FeatureType>::Ptr source_features_;
  std::map<int, std::map<int, int> > segment_object;
  std::map<int, PoseVector> pose_map;

 protected:
  /**
   * @brief remove plane and select largest cluster as input object
   * @param input the input point cloud
   * @param segmented the resulting segmented point cloud containing only points
   * of the largest cluster
   */
  //                    void segmentation (typename
  //                    pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr input,
  //                    typename pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
  //                    segmented) const;
  /**
   * @brief Detects key points in the input point cloud
   * @param input the input point cloud
   * @param keypoints the resulting key points. Note that they are not
   * necessarily a subset of the input cloud
   */
  void detectKeypoints(
      typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr input,
      pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints) const;

  /**
   * @brief extract descriptors for given key points
   * @param input point cloud to be used for descriptor extraction
   * @param keypoints locations where descriptors are to be extracted
   * @param features resulting descriptors
   */
  void extractDescriptors(
      typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr input,
      typename pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints,
      typename pcl::PointCloud<FeatureType>::Ptr features);

  /**
   * @brief find corresponding features based on some metric
   * @param source source feature descriptors
   * @param target target feature descriptors
   * @param correspondences indices out of the target descriptors that
   * correspond (nearest neighbor) to the source descriptors
   */
  void findCorrespondences(typename pcl::PointCloud<FeatureType>::Ptr source,
                           typename pcl::PointCloud<FeatureType>::Ptr target,
                           std::vector<int>& correspondences) const;

  /**
   * @brief  remove non-consistent correspondences
   */
  void filterCorrespondences();

  /**
   * @brief calculate the initial rigid transformation from filtered
   * corresponding keypoints
   */
  void determineInitialTransformation();

  /**
   * @brief calculate the final rigid transformation using ICP over all points
   */
  void determineFinalTransformation();

  /**
   * @brief reconstructs the surface from merged point clouds
   */
  //       void reconstructSurface ();
  /**
   * @brief callback to handle keyboard events
   * @param event object containing information about the event. e.g. type
   * (press, release) etc.
   * @param cookie user defined data passed during registration of the callback
   */

 private:
  //                    pcl::visualization::PCLVisualizer visualizer_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr source_keypoints_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_keypoints_;
  boost::shared_ptr<pcl::Keypoint<pcl::PointXYZRGBA, pcl::PointXYZI> >
      keypoint_detector_;
  typename pcl::Feature<pcl::PointXYZRGBA, FeatureType>::Ptr feature_extractor_;
  // boost::shared_ptr<pcl::PCLSurfaceBase<pcl::PointXYZRGBANormal> >
  // surface_reconstructor_;
  typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr source_;
  typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr target_;
  typename pcl::PointCloud<pcl::PointXYZRGBA>::Ptr source_segmented_;
  typename pcl::PointCloud<pcl::PointXYZRGBA>::Ptr target_segmented_;
  typename pcl::PointCloud<pcl::PointXYZRGBA>::Ptr source_transformed_;
  typename pcl::PointCloud<pcl::PointXYZRGBA>::Ptr source_registered_;
  // typename pcl::PolygonMesh surface_;
  typename pcl::PointCloud<FeatureType>::Ptr target_features_;
  std::vector<int> source2target_;
  std::vector<int> target2source_;
  Eigen::Matrix4f initial_transformation_matrix_;
  Eigen::Matrix4f transformation_matrix_;
  bool show_source2target_;
  bool show_target2source_;
  bool show_correspondences;
  bool verbose_;
  bool debug_;
  boost::shared_ptr<pcl::UniformSampling<pcl::PointXYZRGBA> > uniform_sampler_;

  std::string feature_directory_, base_directory_, pose_directory_,
      model_directory_;

  typename std::map<int, std::vector<pcl::PointCloud<FeatureType> > >
      feature_map;
  std::map<int, std::vector<pcl::PointCloud<pcl::PointXYZI> > > keypoint_map;
  std::map<int, std::vector<Eigen::Vector4f> > centroid_map;
  std::vector<int> object_centroid_map;
  flann::Index<flann::L2<float> >* index;
  std::vector<float> centroid_data;

  // std::map < int, std::map<int, int> > object_segment;

  // std::vector<pcl::PointCloud<PointT> > pcd_files;
  // map<int>
};

//#include <pcl/cloudcv/impl/feature_matches.hpp>

/* OBJECT_RECOGNITION_H_ */
