/*
 * Software License Agreement (BSD License)
 *
 *  OmniMapper
 *  Copyright (c) 2012-, Georgia Tech Research Corporation,
 *  Atlanta, Georgia 30332-0415
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#pragma once

#include <pcl/common/time.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/conversions.h>
#include <boost/thread/locks.hpp>
#include <boost/thread/thread.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>

#include <boost/thread/thread.hpp>
#include <fstream>
#include <iostream>

//#define HAVE_ORGANIZED_EDGES
#ifdef HAVE_ORGANIZED_EDGES
#include <pcl/features/organized_edge_detection.h>
#endif

// Useful macros
#define FPS_CALC(_WHAT_)                                                \
  do {                                                                  \
    static unsigned count = 0;                                          \
    static double last = pcl::getTime();                                \
    double now = pcl::getTime();                                        \
    ++count;                                                            \
    if (now - last >= 1.0) {                                            \
      std::cout << "Average framerate(" << _WHAT_                       \
                << "): " << double(count) / double(now - last) << " Hz" \
                << std::endl;                                           \
      count = 0;                                                        \
      last = now;                                                       \
    }                                                                   \
  } while (false)

namespace cogrob {

/**
 * \brief OrganizedSegmentationTBB concurrently performs various types of
 * segmentation and feature extraction. This includes a pipeline that performs
 * normal estimation -> plane segmentation -> euclidean clustering. Applications
 * include tabletop object detection, and semantic mapping.  Various callbacks
 * are provided for different output formats.
 *
 *  \note If you use plane segmentation or euclidean clustering, please cite:
 *
 *   - A. J. B. Trevor, S. Gedikli, R. Rusu, H. Christensen.
 *     Efficient Organized Point Cloud Segmentation with Connected Components.
 *     3rd Workshop on Semantic Perception, Mapping and Exploration (SPME)
 *     Karlsruhe, Germany, May 5th, 2013.
 *
 *  \note If you use the organized edge detection, please cite:
 *
 *   - C. Choi, A. J. B. Trevor, H. I. Christensen.
 *     RGB-D Edge Detection and Edge Registration.
 *     IROS, 2013.
 *
 * \author Alex Trevor
 */
template <typename PointT>
class OrganizedSegmentationTBB {
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
  boost::optional<
      std::vector<pcl::PlanarRegion<PointT>,
                  Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > >
      mps_output_regions_;
  boost::optional<LabelCloudPtr> mps_output_labels_;
  boost::optional<std::vector<pcl::ModelCoefficients> >
      mps_output_model_coefficients_;
  boost::optional<std::vector<pcl::PointIndices> > mps_output_inlier_indices_;
  boost::optional<std::vector<pcl::PointIndices> > mps_output_label_indices_;
  boost::optional<std::vector<pcl::PointIndices> > mps_output_boundary_indices_;

  // Input to Euclidean Clustering
  boost::optional<CloudConstPtr> clust_input_cloud_;
  boost::optional<
      std::vector<pcl::PlanarRegion<PointT>,
                  Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > >
      clust_input_regions_;
  boost::optional<LabelCloudPtr> clust_input_labels_;
  boost::optional<std::vector<pcl::ModelCoefficients> >
      clust_input_model_coefficients_;
  boost::optional<std::vector<pcl::PointIndices> > clust_input_inlier_indices_;
  boost::optional<std::vector<pcl::PointIndices> > clust_input_label_indices_;
  boost::optional<std::vector<pcl::PointIndices> >
      clust_input_boundary_indices_;

  // Output from Euclidean Clustering
  boost::optional<LabelCloudPtr> clust_output_labels_;
  boost::optional<std::vector<CloudPtr> > clust_output_clusters_;
  boost::optional<std::vector<pcl::PointIndices> >
      clust_output_cluster_indices_;

  // Output from Occluding Edges
  boost::optional<CloudPtr> oed_output_occluding_edge_cloud_;

  // Publishing
  boost::optional<LabelCloudPtr> pub_cluster_labels_;
  boost::optional<CloudConstPtr> pub_cluster_cloud_;
  boost::optional<CloudPtr> pub_occluding_edge_cloud_;
  boost::optional<std::vector<CloudPtr> > pub_clusters_;
  boost::optional<std::vector<pcl::PointIndices> > pub_cluster_indices_;
  boost::optional<
      std::vector<pcl::PlanarRegion<PointT>,
                  Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > >
      pub_mps_regions_;

  boost::mutex cloud_mutex;

  bool updated_cloud_;

  // Normal Estimation
  boost::shared_ptr<pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> >
      ne_;

  // Plane Segmentation
  boost::shared_ptr<
      pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> >
      mps_;

  // Objects
  typename pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>::Ptr
      euclidean_cluster_comparator_;

// Edge Detection
#ifdef HAVE_ORGANIZED_EDGE
  pcl::OrganizedEdgeFromRGBNormals<PointT, pcl::Normal, pcl::Label> oed;
#endif

  // Planar Region Callback
  boost::function<void(
      std::vector<pcl::PlanarRegion<PointT>,
                  Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >&)>
      planar_region_callback_;

  // Planar Region Stamped Callback
  boost::function<void(
      std::vector<pcl::PlanarRegion<PointT>,
                  Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >,
      Time)>
      planar_region_stamped_callback_;

  std::vector<boost::function<void(
      std::vector<pcl::PlanarRegion<PointT>,
                  Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >&,
      Time&)> >
      planar_region_stamped_callbacks_;

  // Planar Regions with inliers (for visualization, usually).  Cloud, models,
  // inliers, label indices, boundary indices
  std::vector<boost::function<void(
      const CloudConstPtr, Time t, std::vector<pcl::ModelCoefficients>,
      std::vector<pcl::PointIndices>, std::vector<pcl::PointIndices>,
      std::vector<pcl::PointIndices>)> >
      full_planar_callbacks_;

  // Edge Callbacks
  boost::function<void(const CloudConstPtr&)> occluding_edge_callback_;

  // Plane Label Callback
  boost::function<void(const CloudConstPtr&, const LabelCloudConstPtr&)>
      plane_label_cloud_callback_;

  // Cluster Label Callback
  std::vector<
      boost::function<void(const CloudConstPtr&, const LabelCloudConstPtr&)> >
      cluster_label_cloud_callbacks_;

  // RegionCloud Callback
  boost::function<void(
      const CloudConstPtr&,
      std::vector<pcl::PlanarRegion<PointT>,
                  Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >&)>
      region_cloud_callback_;

  // Cluster Cloud Callback
  std::vector<
      boost::function<void(std::vector<CloudPtr>, Time t,
                           boost::optional<std::vector<pcl::PointIndices> >)> >
      cluster_cloud_callbacks_;

  // Cluster Cloud Indices Callback
  // std::vector<boost::function<void(std::vector<CloudPtr>,
  // std::vector<pcl::PointIndices>, Time t)> > cluster_cloud_indices_callbacks_;
  std::vector<boost::function<void(const CloudConstPtr, Time,
                                   std::vector<pcl::PointIndices>)> >
      full_cluster_callbacks_;

  // Set min plane inliers
  void setMinPlaneInliers(int min_inliers) {
    min_plane_inliers_ = min_inliers;
    mps_->setMinInliers(min_inliers);
  }

  // Set min cluster inliers
  void setMinClusterInliers(int min_inliers) {
    min_cluster_inliers_ = min_inliers;
  }

  // Threads
  boost::thread spin_thread;

  // Parameters
  int min_plane_inliers_;
  int min_cluster_inliers_;

  // Flags
  bool debug_;
  bool timing_;
  bool ready_;

  // Output
  std::ofstream ne_times_file_;
  std::ofstream mps_times_file_;

 private:
  void spinThread();

  /* \brief Pipeline stage: computes the surface normals. */
  void computeNormals();

  /* \brief Pipeline stage: performs plane segmentation. */
  void computePlanes();

  /* \brief Pipeline stage: performs euclidean clustering on remaining
   * non-planar areas. */
  void computeClusters();

  /* \brief Feature extraction: Computes edges. */
#ifdef HAVE_ORGANIZED_EDGE
  void computeEdges();
#endif

  /* \brief Pipeline stage: publishes output to callbacks. */
  void publish();

  /* \brief converts a PCL stamp to a boost posix time. */
  static Time stamp2ptime(uint64_t stamp);

 public:
  /* \brief Default Constructor. */
  OrganizedSegmentationTBB();

  /* \brief Input is provided to the segmentation class through this callback.
   */
  void cloudCallback(const CloudConstPtr& cloud);

  /* \brief Returns false if any pipeline stage is currently processing, true
   * otherwise. */
  bool ready();

  /* \brief Call once to start a processing thread for the pipeline.
   * Non-blocking. */
  void spin();

  /* \brief Call to process one step of the pipeline. */
  void spinOnce();

  /* \brief Installs a planar region callback. */
  void setPlanarRegionCallback(
      boost::function<void(
          std::vector<pcl::PlanarRegion<PointT>,
                      Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >&)>&
          fn);

  /* \brief Installs a planar region callback with timestamp. */
  void setPlanarRegionStampedCallback(
      boost::function<void(
          std::vector<pcl::PlanarRegion<PointT>,
                      Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >,
          Time)>& fn);

  /* \brief Installs a plane callback with all output information. */
  void setFullPlanarRegionCallback(
      boost::function<
          void(const CloudConstPtr, Time, std::vector<pcl::ModelCoefficients>,
               std::vector<pcl::PointIndices>, std::vector<pcl::PointIndices>,
               std::vector<pcl::PointIndices>)>& fn);

  /* \brief Installs an occluding edge callback. */
  void setOccludingEdgeCallback(
      boost::function<void(const CloudConstPtr&)>& fn);

  /* \brief Installs a callback for the raw planar labels, used by plane
   * segmentation. */
  void setPlaneLabelsCallback(
      boost::function<void(const CloudConstPtr&, const LabelCloudConstPtr&)>&
          fn);

  /* \brief Installs a callback for the raw cluster labels, used by euclidean
   * clustering. */
  void setClusterLabelsCallback(
      boost::function<void(const CloudConstPtr&, const LabelCloudConstPtr&)>&
          fn);

  /* \brief Installs a callback for planar regions withthe cloud they were
   * extracted from. */
  void setRegionCloudCallback(
      boost::function<void(
          const CloudConstPtr&,
          std::vector<pcl::PlanarRegion<PointT>,
                      Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >&)>&
          fn);

  /* \brief Installs a callback for clusters with point indices. */
  void setClusterCloudCallback(
      boost::function<void(std::vector<CloudPtr>, Time,
                           boost::optional<std::vector<pcl::PointIndices> >)>
          fn);

  /* \brief Installs a cluster callback with all output information. */
  void setFullClusterCallback(
      boost::function<void(const CloudConstPtr, Time,
                           std::vector<pcl::PointIndices>)>& fn);

  /* \brief Returns a shared pointer to the normal estimation. */
  boost::shared_ptr<pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> >
  getNormalEstimation() {
    return (ne_);
  }

  /* \brief Returns a shared point to the plane segmentation. */
  boost::shared_ptr<
      pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> >
  getPlaneSegmentation() {
    return (mps_);
  }

  /* \brief Returns a shared pointer to the euclidean clustering comparator. */
  boost::shared_ptr<
      pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label> >
  getClusterComparator() {
    return (euclidean_cluster_comparator_);
  }
};

}  // namespace cogrob
