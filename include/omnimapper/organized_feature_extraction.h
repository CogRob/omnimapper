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

/** \brief OrganizedFeatureExtraction is our helper class for handling multiple types of feature extraction.
  *
  * \author Alex Trevor
  */
template <typename PointT>
class OrganizedFeatureExtraction
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
    // PCL Grabber
    pcl::Grabber& grabber_;

    // Most recent cloud from the sensor
    CloudConstPtr prev_sensor_cloud_;
    boost::mutex sensor_cloud_mutex;

    // We process normals for frame n-1 concurrently with feature extraction for cloud n
    CloudConstPtr stage1_cloud_;
    NormalCloudPtr stage1_normals_;
    CloudConstPtr stage2_cloud_;
    CloudConstPtr stage3_cloud_;
    CloudConstPtr stage4_cloud_;
    CloudConstPtr dummy_cloud_;
    NormalCloudConstPtr stage2_normals_;
    std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > stage3_regions_;
    std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > stage4_regions_;
    LabelCloudPtr stage3_labels_;
    LabelCloudPtr stage4_labels_;
    LabelCloudPtr stage5_labels_;
    CloudPtr stage3_occluding_cloud_;
    std::vector<pcl::ModelCoefficients> stage3_model_coefficients_;
    std::vector<pcl::PointIndices> stage3_inlier_indices_;
    std::vector<pcl::PointIndices> stage3_label_indices_;
    std::vector<pcl::PointIndices> stage3_boundary_indices_;
    std::vector<pcl::ModelCoefficients> stage4_model_coefficients_;
    std::vector<pcl::PointIndices> stage4_inlier_indices_;
    std::vector<pcl::PointIndices> stage4_label_indices_;
    std::vector<pcl::PointIndices> stage4_boundary_indices_;
    std::vector<CloudPtr> stage5_clusters_;
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
    pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;

    // Plane Segmentation
    pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;

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

    // Label Callback
    boost::function<void(const CloudConstPtr&, const LabelCloudConstPtr&)> label_cloud_callback_;

    // Cluster Label Callback
    std::vector<boost::function<void(const CloudConstPtr&, const LabelCloudConstPtr&)> > cluster_label_cloud_callbacks_;

    // RegionCloud Callback
    boost::function<void(const CloudConstPtr&, std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >&)> region_cloud_callback_;    
    
    // Cluster Cloud Callback
    std::vector<boost::function<void(std::vector<CloudPtr>, Time t)> > cluster_cloud_callbacks_;

    // Threads
    boost::thread vis_thread;
    boost::thread process_thread;

    // Flags
    bool debug_;
    bool timing_;

    // Output
    std::ofstream ne_times_file_;
    std::ofstream mps_times_file_;

  public:
    OrganizedFeatureExtraction (pcl::Grabber& grabber);
    
    void cloudCallback (const CloudConstPtr& cloud);
    void cloudCallbackProcess (const CloudConstPtr& cloud);
    void processFrame ();
    void computeNormals ();
    void computePlanes ();
    void computeClusters ();
    void computeEdges ();
    void spin ();
    void setPlanarRegionCallback (boost::function<void (std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >&)>& fn);
    void setPlanarRegionStampedCallback (boost::function<void (std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >, Time)>& fn);
    void setOccludingEdgeCallback (boost::function<void (const CloudConstPtr&)>& fn);
    void setLabelsCallback (boost::function<void (const CloudConstPtr&, const LabelCloudConstPtr&)>& fn);
    void setClusterLabelsCallback (boost::function<void (const CloudConstPtr&, const LabelCloudConstPtr&)>& fn);
    void setRegionCloudCallback (boost::function<void(const CloudConstPtr&, std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >&)>& fn);
    void setClusterCloudCallback (boost::function<void(std::vector<CloudPtr>, Time)> fn);
    
};

}
