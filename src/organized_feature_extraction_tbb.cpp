#include <glog/logging.h>
#include <omnimapper/organized_feature_extraction_tbb.h>
#include <omnimapper/time.h>
#include <pcl/segmentation/plane_refinement_comparator.h>
#include <tbb/parallel_invoke.h>
#include <tbb/pipeline.h>
#include <tbb/spin_mutex.h>
#include <tbb/task.h>
#include <tbb/task_group.h>
#include <tbb/task_scheduler_init.h>
#include <tbb/tbb_thread.h>

namespace omnimapper {

template <typename PointT>
OrganizedFeatureExtractionTBB<PointT>::OrganizedFeatureExtractionTBB()
    : input_cloud_(boost::none),
      ne_output_normals_(boost::none),
      ne_output_cloud_(boost::none),
      mps_input_cloud_(boost::none),
      mps_input_normals_(boost::none),
      mps_output_cloud_(boost::none),
      mps_output_regions_(boost::none),
      mps_output_labels_(new LabelCloud()),
      clust_input_cloud_(boost::none),
      clust_input_labels_(boost::none),
      pub_cluster_labels_(boost::none),
      pub_cluster_cloud_(boost::none),
      pub_occluding_edge_cloud_(boost::none),
      pub_mps_regions_(boost::none),
      stage4_labels_(new LabelCloud()),
      clust_output_labels_(boost::none),
      oed_output_occluding_edge_cloud_(boost::none),
      vis_cloud_(new Cloud()),
      vis_labels_(new LabelCloud()),
      vis_occluding_cloud_(new Cloud()),
      vis_normals_(new NormalCloud()),
      updated_data_(false),
      updated_cloud_(false),
      ne_(new pcl::IntegralImageNormalEstimation<PointT, pcl::Normal>()),
      mps_(new pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal,
                                                    pcl::Label>()),
      euclidean_cluster_comparator_(
          new pcl::EuclideanClusterComparator<PointT, pcl::Normal,
                                              pcl::Label>()),
      min_plane_inliers_(10000),
      min_cluster_inliers_(1000),
      debug_(false),
      timing_(false),
      ready_(true) {
  LOG_IF(INFO, debug_) << "OrganizedFeatureExtractionTBB start.";

  // Set up Normal Estimation
  // This could also be ne_->SIMPLE_3D_GRADIENT
  ne_->setNormalEstimationMethod(ne_->COVARIANCE_MATRIX);
  ne_->setMaxDepthChangeFactor(0.02f);
  ne_->setNormalSmoothingSize(20.0f);

  // Set up plane segmentation
  mps_->setMinInliers(min_plane_inliers_);
  mps_->setAngularThreshold(pcl::deg2rad(2.0));
  mps_->setDistanceThreshold(0.02);
  mps_->setProjectPoints(true);
  mps_->setRemoveDuplicatePoints(false);
  pcl::PlaneCoefficientComparator<pcl::PointXYZRGBA, pcl::Normal>::Ptr
      plane_compare(new pcl::PlaneCoefficientComparator<pcl::PointXYZRGBA,
                                                        pcl::Normal>());
  plane_compare->setAngularThreshold(pcl::deg2rad(2.0));
  plane_compare->setDistanceThreshold(0.01, true);
  mps_->setComparator(plane_compare);

  pcl::PlaneRefinementComparator<pcl::PointXYZRGBA, pcl::Normal,
                                 pcl::Label>::Ptr
      refine_compare(
          new pcl::PlaneRefinementComparator<pcl::PointXYZRGBA, pcl::Normal,
                                             pcl::Label>());
  refine_compare->setDistanceThreshold(0.005, false);
  mps_->setRefinementComparator(refine_compare);

  // Set up edge detection
  oed_.setDepthDisconThreshold(0.04f);
  oed_.setMaxSearchNeighbors(100);
  oed_.setEdgeType(oed_.EDGELABEL_NAN_BOUNDARY | oed_.EDGELABEL_OCCLUDING |
                   oed_.EDGELABEL_OCCLUDED);
  // This could also be oed.EDGELABEL_RGB_CANNY

  // Set up output
  if (timing_) {
    ne_times_file_.open("/tmp/ne_times.txt");
    mps_times_file_.open("/tmp/mps_times.txt");
  }
}

// Get latest cloud from the sensor
template <typename PointT>
void OrganizedFeatureExtractionTBB<PointT>::CloudCallback(
    const CloudConstPtr& cloud) {
  // Store cloud
  boost::mutex::scoped_lock(cloud_mutex);
  {
    FPS_CALC("cloud_callback");
    prev_sensor_cloud_ = cloud;
    updated_cloud_ = true;
    ready_ = false;
  }
}

template <typename PointT>
bool OrganizedFeatureExtractionTBB<PointT>::Ready() {
  boost::mutex::scoped_lock lock(cloud_mutex_);
  return (ready_);
}

template <typename PointT>
void OrganizedFeatureExtractionTBB<PointT>::Spin() {
  spin_thread_ =
      boost::thread(&OrganizedFeatureExtractionTBB<PointT>::SpinThread, this);
}

template <typename PointT>
void OrganizedFeatureExtractionTBB<PointT>::SpinThread() {
  while (true) {
    boost::this_thread::sleep(boost::posix_time::milliseconds(5));
    SpinOnce();
  }
}

template <typename PointT>
void OrganizedFeatureExtractionTBB<PointT>::SpinOnce() {
  FPS_CALC("spin");

  const double frame_start = pcl::getTime();
  // Get latest cloud
  input_cloud_ = boost::none;
  if (cloud_mutex_.try_lock()) {
    if (updated_cloud_) {
      input_cloud_ = prev_sensor_cloud_;
      updated_cloud_ = false;
      ready_ = false;
    }
    cloud_mutex_.unlock();
  }

  // Normal Estimation && MPS && Clustering
  tbb::task_group group;
  group.run([&] { ComputeNormals(); });
  group.run([&] { ComputePlanes(); });
  if ((cluster_cloud_callbacks_.size() > 0) ||
      (cluster_label_cloud_callbacks_.size() > 0))
    group.run([&] { ComputeClusters(); });
  if (occluding_edge_callback_) group.run([&] { ComputeEdges(); });
  group.run([&] { Publish(); });
  group.wait();

  // Move clouds along pipeline
  pub_cluster_cloud_ = clust_input_cloud_;
  pub_cluster_labels_ = clust_output_labels_;
  pub_clusters_ = clust_output_clusters_;
  pub_cluster_indices_ = clust_output_cluster_indices_;
  pub_occluding_edge_cloud_ = oed_output_occluding_edge_cloud_;
  pub_mps_regions_ = mps_output_regions_;

  clust_input_cloud_ = mps_input_cloud_;
  clust_input_regions_ = mps_output_regions_;
  clust_input_labels_ = mps_output_labels_;
  clust_input_model_coefficients_ = mps_output_model_coefficients_;
  clust_input_inlier_indices_ = mps_output_inlier_indices_;
  clust_input_label_indices_ = mps_output_label_indices_;
  clust_input_boundary_indices_ = mps_output_boundary_indices_;

  mps_input_normals_ = ne_output_normals_;
  mps_input_cloud_ = input_cloud_;

  if (!ne_output_normals_ && !mps_output_labels_ && !clust_output_labels_ &&
      !pub_cluster_labels_ && !oed_output_occluding_edge_cloud_ &&
      !pub_occluding_edge_cloud_) {
    boost::mutex::scoped_lock(cloud_mutex);
    { ready_ = true; }
  }

  const double frame_end = pcl::getTime();
  LOG_IF(INFO, debug_) << "Frame took: " << double(frame_end - frame_start)
                       << std::endl;
}

template <typename PointT>
void OrganizedFeatureExtractionTBB<PointT>::Publish() {
  LOG_IF(INFO, debug_) << "OrganizedFeatureExtraction Publishing.";
  // Publish plane Labels
  if (plane_label_cloud_callback_) {
    if (clust_input_cloud_ && clust_input_labels_) {
      LOG_IF(INFO, debug_) << "Publish plane labels.";
      plane_label_cloud_callback_(*clust_input_cloud_, *clust_input_labels_);
    }
  }

  // Publish plane regions
  if (planar_region_stamped_callbacks_.size() > 0) {
    if (pub_mps_regions_) {
      LOG_IF(INFO, debug_) << "Publish plane regions.";
      Time timestamp = StampToPtime((*clust_input_cloud_)->header.stamp);
      for (std::size_t i = 0; i < planar_region_stamped_callbacks_.size(); i++)
        planar_region_stamped_callbacks_[i](*pub_mps_regions_, timestamp);
    }
  }

  // Publish Cluster Labels
  if (cluster_label_cloud_callbacks_.size() > 0) {
    if (pub_cluster_cloud_ && pub_cluster_labels_) {
      LOG_IF(INFO, debug_) << "Publish cluster labels.";
      for (std::size_t i = 0; i < cluster_label_cloud_callbacks_.size(); i++)
        cluster_label_cloud_callbacks_[i](*pub_cluster_cloud_,
                                          *pub_cluster_labels_);
    }
  }

  // Publish Cluster Clouds
  if (cluster_cloud_callbacks_.size() > 0) {
    LOG_IF(INFO, debug_) << "Publish cluster clouds.";
    if (pub_cluster_cloud_) {
      Time timestamp = StampToPtime((*pub_cluster_cloud_)->header.stamp);
      for (std::size_t i = 0; i < cluster_cloud_callbacks_.size(); i++) {
        LOG_IF(INFO, debug_) << "Publish cluster clouds with callback " << i;
        cluster_cloud_callbacks_[i](*pub_clusters_, timestamp,
                                    *pub_cluster_indices_);
      }
    }
  }

  // Publish Occluding Edges
  if (occluding_edge_callback_) {
    LOG_IF(INFO, debug_) << "Have occ edge callbacks!";
    if (pub_occluding_edge_cloud_) {
      LOG_IF(INFO, debug_) << "Publish occluding edges.";
      occluding_edge_callback_(*pub_occluding_edge_cloud_);
    }
  }
}

// Compute the normals
template <typename PointT>
void OrganizedFeatureExtractionTBB<PointT>::ComputeNormals() {
  if (!input_cloud_) {
    ne_output_cloud_ = boost::none;
    ne_output_normals_ = boost::none;
    return;
  }
  ne_->setInputCloud(*input_cloud_);
  double start = pcl::getTime();
  NormalCloudPtr normals(new NormalCloud());

  ne_->compute(*normals);
  ne_output_normals_ = normals;
  double end = pcl::getTime();
  if (timing_) {
    ne_times_file_ << double(end - start) << std::endl;
    LOG_IF(INFO, debug_) << double(end - start);
  }
  LOG_IF(INFO, debug_) << "NE took : " << double(end - start);
  return;
}

// Compute clusters
template <typename PointT>
void OrganizedFeatureExtractionTBB<PointT>::ComputeClusters() {
  if (!(clust_input_labels_ && clust_input_cloud_)) return;
  // Segment Objects
  std::vector<CloudPtr> output_clusters;
  std::vector<pcl::PointIndices> output_cluster_indices;
  clust_output_clusters_ = output_clusters;
  clust_output_cluster_indices_ = output_cluster_indices;
  clust_output_clusters_->clear();
  clust_output_cluster_indices_->clear();

  clust_output_labels_ = LabelCloudPtr(new LabelCloud());

  std::vector<bool> plane_labels;
  plane_labels.resize(clust_input_label_indices_->size(), false);

  if (clust_input_regions_->size() > 0) {
    for (size_t i = 0; i < clust_input_label_indices_->size(); i++) {
      if (static_cast<int>((*clust_input_label_indices_)[i].indices.size()) >
          min_plane_inliers_) {
        plane_labels[i] = true;
      }
    }
  }

  euclidean_cluster_comparator_->setInputCloud(*clust_input_cloud_);
  euclidean_cluster_comparator_->setLabels(*clust_input_labels_);
  euclidean_cluster_comparator_->setExcludeLabels(plane_labels);
  euclidean_cluster_comparator_->setDistanceThreshold(0.01f, false);

  pcl::PointCloud<pcl::Label> euclidean_labels;
  std::vector<pcl::PointIndices> euclidean_label_indices;
  pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label>
      euclidean_segmentation(euclidean_cluster_comparator_);
  euclidean_segmentation.setInputCloud(*clust_input_cloud_);
  LOG_IF(INFO, debug_) << "Calling segment for euclidean clusters.";
  euclidean_segmentation.segment(*(*clust_output_labels_),
                                 euclidean_label_indices);
  LOG_IF(INFO, debug_) << "Done with segment for euclidean clusters.";

  for (size_t i = 0; i < euclidean_label_indices.size(); i++) {
    if (euclidean_label_indices[i].indices.size() > 1000) {
      CloudPtr cluster(new Cloud());
      pcl::copyPointCloud(*(*clust_input_cloud_),
                          euclidean_label_indices[i].indices, *cluster);

      clust_output_clusters_->push_back(cluster);
      clust_output_cluster_indices_->push_back(euclidean_label_indices[i]);
    }
  }

  LOG_IF(INFO, debug_) << "Got " << clust_output_clusters_->size()
                       << " euclidean clusters!";
}

// Compute planes
template <typename PointT>
void OrganizedFeatureExtractionTBB<PointT>::ComputePlanes() {
  if ((!mps_input_cloud_) || (!mps_input_normals_)) {
    mps_output_labels_ = boost::none;
    mps_output_regions_ = boost::none;
    return;
  }
  if ((*mps_input_cloud_)->points.size() == 0) {
    mps_output_labels_ = boost::none;
    mps_output_regions_ = boost::none;
    return;
  }

  mps_->setInputNormals(*mps_input_normals_);
  mps_->setInputCloud(*mps_input_cloud_);

  double init_start = pcl::getTime();
  std::vector<pcl::ModelCoefficients> model_coefficients;
  std::vector<pcl::PointIndices> inlier_indices;
  std::vector<pcl::PointIndices> label_indices;
  std::vector<pcl::PointIndices> boundary_indices;
  mps_output_model_coefficients_ = model_coefficients;
  mps_output_inlier_indices_ = inlier_indices;
  mps_output_label_indices_ = label_indices;
  mps_output_boundary_indices_ = boundary_indices;
  mps_output_labels_ = LabelCloudPtr(new LabelCloud());
  mps_output_regions_ =
      std::vector<pcl::PlanarRegion<PointT>,
                  Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >();

  double start = pcl::getTime();
  LOG_IF(INFO, debug_) << "refine";

  mps_->segmentAndRefine(
      (*mps_output_regions_), (*mps_output_model_coefficients_),
      (*mps_output_inlier_indices_), (*mps_output_labels_),
      (*mps_output_label_indices_), (*mps_output_boundary_indices_));

  double end = pcl::getTime();
  LOG_IF(INFO, debug_) << "mps segment and refine took: "
                       << double(end - start);
  if (timing_) {
    mps_times_file_ << double(end - start) << std::endl;
    LOG_IF(INFO, debug_) << double(end - start);
  }
  if (mps_output_regions_)
    LOG_IF(INFO, debug_) << "Got " << mps_output_regions_->size()
                         << " regions!";
}

// Extract edges
template <typename PointT>
void OrganizedFeatureExtractionTBB<PointT>::ComputeEdges() {
  if (!input_cloud_) {
    LOG_IF(INFO, debug_) << "edges returning early";
    oed_output_occluding_edge_cloud_ = boost::none;
    return;
  }

  double edge_start = pcl::getTime();
  pcl::PointCloud<pcl::Label> labels;
  std::vector<pcl::PointIndices> label_indices;
  oed_.setInputCloud(*input_cloud_);

  oed_.compute(labels, label_indices);
  double edge_end = pcl::getTime();
  LOG_IF(INFO, debug_) << "edges took: " << double(edge_end - edge_start);
  CloudPtr edge_cloud(new Cloud());
  pcl::copyPointCloud(*(*input_cloud_), label_indices[1], *edge_cloud);
  oed_output_occluding_edge_cloud_ = edge_cloud;
}

template <typename PointT>
void OrganizedFeatureExtractionTBB<PointT>::SetPlanarRegionCallback(
    boost::function<void(
        std::vector<pcl::PlanarRegion<PointT>,
                    Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >&)>&
        fn) {
  planar_region_callback_ = fn;
}

template <typename PointT>
void OrganizedFeatureExtractionTBB<PointT>::SetPlanarRegionStampedCallback(
    boost::function<
        void(std::vector<pcl::PlanarRegion<PointT>,
                         Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >,
             Time)>& fn) {
  planar_region_stamped_callbacks_.push_back(fn);
  LOG_IF(INFO, debug_) << "planar region stamped callback set!.";
}

template <typename PointT>
void OrganizedFeatureExtractionTBB<PointT>::SetOccludingEdgeCallback(
    boost::function<void(const CloudConstPtr&)>& fn) {
  occluding_edge_callback_ = fn;
}

template <typename PointT>
void OrganizedFeatureExtractionTBB<PointT>::SetPlaneLabelsCallback(
    boost::function<void(const CloudConstPtr&, const LabelCloudConstPtr&)>&
        fn) {
  plane_label_cloud_callback_ = fn;
}

template <typename PointT>
void OrganizedFeatureExtractionTBB<PointT>::SetClusterLabelsCallback(
    boost::function<void(const CloudConstPtr&, const LabelCloudConstPtr&)>&
        fn) {
  cluster_label_cloud_callbacks_.push_back(fn);
}

template <typename PointT>
void OrganizedFeatureExtractionTBB<PointT>::SetRegionCloudCallback(
    boost::function<void(
        const CloudConstPtr&,
        std::vector<pcl::PlanarRegion<PointT>,
                    Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >&)>&
        fn) {
  region_cloud_callback_ = fn;
}

template <typename PointT>
void OrganizedFeatureExtractionTBB<PointT>::SetClusterCloudCallback(
    boost::function<void(std::vector<CloudPtr>, Time,
                         boost::optional<std::vector<pcl::PointIndices> >)>
        fn) {
  cluster_cloud_callbacks_.push_back(fn);
}

}  // namespace omnimapper

// Instantiate
template class omnimapper::OrganizedFeatureExtractionTBB<pcl::PointXYZRGBA>;
