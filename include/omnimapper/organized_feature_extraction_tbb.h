#pragma once

// Boost
#include <boost/thread/thread.hpp>
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/features/organized_edge_detection.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/common/time.h>
#include <iostream>
#include <fstream>



// Useful macros
#define FPS_CALC(_WHAT_) \
do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime ();\
    double now = pcl::getTime (); \
    ++count; \
    if (now - last >= 1.0) \
    { \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
      count = 0; \
      last = now; \
    } \
}while(false)

//typedef pcl::PointXYZRGBA PointT;

namespace omnimapper
{

/** \brief OrganizedFeatureExtraction concurrently performs various types of feature extraction.
  *
  * \author Alex Trevor
  */
template <typename PointT>
class OrganizedFeatureExtractionTBB
{
  typedef pcl::PointCloud<PointT> Cloud;
  typedef typename Cloud::Ptr CloudPtr;
  typedef typename Cloud::ConstPtr CloudConstPtr;
  typedef pcl::PointCloud<pcl::Normal> NormalCloud;
  typedef typename NormalCloud::Ptr NormalCloudPtr;
  typedef typename NormalCloud::ConstPtr NormalCloudConstPtr;
  typedef pcl::PointCloud<pcl::Label> LabelCloud;
  typedef typename LabelCloud::Ptr LabelCloudPtr;
  typedef typename LabelCloud::ConstPtr LabelCloudConstPtr;
  typedef boost::posix_time::ptime Time;

  protected:
    // Most recent cloud from the sensor
    CloudConstPtr prev_sensor_cloud_;
    boost::mutex sensor_cloud_mutex;

    // Latest cloud from the sensor
    boost::optional<CloudConstPtr> input_cloud_;

    // Output from Normal Estimation
    boost::optional<NormalCloudPtr> ne_output_normals_;
    boost::optional<CloudConstPtr> ne_output_cloud_;

    // Input to Multi Plane Segmentation
    boost::optional<CloudConstPtr> mps_input_cloud_;
    boost::optional<NormalCloudConstPtr> mps_input_normals_;

    // Output from Multi Plane Segmentation
    boost::optional<CloudConstPtr> mps_output_cloud_;
    boost::optional<std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > > mps_output_regions_;
    boost::optional<LabelCloudPtr> mps_output_labels_;
    boost::optional<std::vector<pcl::ModelCoefficients> > mps_output_model_coefficients_;
    boost::optional<std::vector<pcl::PointIndices> > mps_output_inlier_indices_;
    boost::optional<std::vector<pcl::PointIndices> > mps_output_label_indices_;
    boost::optional<std::vector<pcl::PointIndices> > mps_output_boundary_indices_;
    
    // Input to Euclidean Clustering
    boost::optional<CloudConstPtr> clust_input_cloud_;
    boost::optional<std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > > clust_input_regions_;
    boost::optional<LabelCloudPtr> clust_input_labels_;
    boost::optional<std::vector<pcl::ModelCoefficients> > clust_input_model_coefficients_;
    boost::optional<std::vector<pcl::PointIndices> > clust_input_inlier_indices_;
    boost::optional<std::vector<pcl::PointIndices> > clust_input_label_indices_;
    boost::optional<std::vector<pcl::PointIndices> > clust_input_boundary_indices_;

    boost::optional<LabelCloudPtr> pub_cluster_labels_;
    boost::optional<CloudConstPtr> pub_cluster_cloud_;
    boost::optional<std::vector<CloudPtr> > pub_clusters_;
    boost::optional<std::vector<pcl::PointIndices> > pub_cluster_indices_;
    
    boost::optional<CloudConstPtr> stage4_cloud_;
    CloudConstPtr dummy_cloud_;


    boost::optional<std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > > stage4_regions_;

    boost::optional<LabelCloudPtr> stage4_labels_;
    boost::optional<LabelCloudPtr> clust_output_labels_;
    boost::optional<CloudPtr> oed_output_occluding_edge_cloud_;

    boost::optional<std::vector<pcl::ModelCoefficients> > stage4_model_coefficients_;
    boost::optional<std::vector<pcl::PointIndices> > stage4_inlier_indices_;
    boost::optional<std::vector<pcl::PointIndices> > stage4_label_indices_;
    boost::optional<std::vector<pcl::PointIndices> > stage4_boundary_indices_;
    boost::optional<std::vector<CloudPtr> > clust_output_clusters_;
    boost::optional<std::vector<pcl::PointIndices> > clust_output_cluster_indices_;
    boost::mutex cloud_mutex;

    // Most recently processed cloud
    CloudConstPtr vis_cloud_;
    LabelCloudConstPtr vis_labels_;
    CloudConstPtr vis_occluding_cloud_;
    NormalCloudConstPtr vis_normals_;
    std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > vis_regions_;    
    boost::mutex vis_mutex;
    boost::mutex state_mutex;
    bool updated_data_;
    bool updated_cloud_;
    boost::condition_variable updated_cond_;

    // Normal Estimation
    boost::shared_ptr<pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> > ne_;

    // Plane Segmentation
    boost::shared_ptr<pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> > mps_;

    // Objects
    typename pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>::Ptr euclidean_cluster_comparator_;

    // Edge Detection
    pcl::OrganizedEdgeFromRGBNormals<PointT, pcl::Normal, pcl::Label> oed;

    // Planar Region Callback
    boost::function<void(std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >&)> planar_region_callback_;

    // Planar Region Stamped Callback
    boost::function<void(std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >, Time)> planar_region_stamped_callback_;

    std::vector<boost::function<void(std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >&, Time&)> > planar_region_stamped_callbacks_;

    // Edge Callbacks
    boost::function<void(const CloudConstPtr&)> occluding_edge_callback_;

    // Plane Label Callback
    boost::function<void(const CloudConstPtr&, const LabelCloudConstPtr&)> plane_label_cloud_callback_;

    // Cluster Label Callback
    std::vector<boost::function<void(const CloudConstPtr&, const LabelCloudConstPtr&)> > cluster_label_cloud_callbacks_;

    // RegionCloud Callback
    boost::function<void(const CloudConstPtr&, std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >&)> region_cloud_callback_;    
    
    // Cluster Cloud Callback
    std::vector<boost::function<void(std::vector<CloudPtr>, Time t, boost::optional<std::vector<pcl::PointIndices> > )> > cluster_cloud_callbacks_;

    // Cluster Cloud Indices Callback
    std::vector<boost::function<void(std::vector<CloudPtr>, std::vector<pcl::PointIndices>, Time t)> > cluster_cloud_indices_callbacks_;

    // Set min plane inliers
    void setMinPlaneInliers (int min_inliers) { min_plane_inliers_ = min_inliers; mps_->setMinInliers (min_inliers); }
    
    // Set min cluster inliers
    void setMinClusterInliers (int min_inliers) { min_cluster_inliers_ = min_inliers; }

    // Threads
    boost::thread vis_thread;
    boost::thread spin_thread;

    // Parameters
    int min_plane_inliers_;
    int min_cluster_inliers_;

    // Flags
    bool debug_;
    bool timing_;

    // Output
    std::ofstream ne_times_file_;
    std::ofstream mps_times_file_;

  private:
    void spinThread ();

  public:
    OrganizedFeatureExtractionTBB ();
    
    void cloudCallback (const CloudConstPtr& cloud);
    void computeNormals ();
    void computePlanes ();
    void publish ();
    void computeClusters ();
    void computeEdges ();
    void spin ();
    //void tbbSpin ();
    void spinOnce();
    void setPlanarRegionCallback (boost::function<void (std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >&)>& fn);
    void setPlanarRegionStampedCallback (boost::function<void (std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >, Time)>& fn);
    void setOccludingEdgeCallback (boost::function<void (const CloudConstPtr&)>& fn);
    void setPlaneLabelsCallback (boost::function<void (const CloudConstPtr&, const LabelCloudConstPtr&)>& fn);
    void setClusterLabelsCallback (boost::function<void (const CloudConstPtr&, const LabelCloudConstPtr&)>& fn);
    void setRegionCloudCallback (boost::function<void(const CloudConstPtr&, std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >&)>& fn);
    void setClusterCloudCallback (boost::function<void(std::vector<CloudPtr>, Time,  boost::optional<std::vector<pcl::PointIndices> > )> fn);
    
};

}
