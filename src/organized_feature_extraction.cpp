#include <omnimapper/organized_feature_extraction.h>
#include <omnimapper/time.h>
#include <pcl/segmentation/plane_refinement_comparator.h>

// // Boost
// #include <boost/thread/thread.hpp>
// // PCL
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/io/openni_grabber.h>
// #include <pcl/features/integral_image_normal.h>
// #include <pcl/segmentation/organized_multi_plane_segmentation.h>
// #include <pcl/features/organized_edge_detection.h>
// #include <pcl/common/time.h>

//#include <pcl/features/impl/organized_edge_detection.hpp>
// Need to instantiate a thing here to use XYZ Point Type with OFE
// pcl::features::OrganizedEdgeFromRGBNormals<pcl::PointXYZ, pcl::Normal,
// pcl::Label>;

namespace omnimapper {
template <typename PointT>
OrganizedFeatureExtraction<PointT>::OrganizedFeatureExtraction(
    pcl::Grabber& grabber)
    : grabber_(grabber),
      prev_sensor_cloud_(new Cloud()),
      stage1_cloud_(new Cloud()),
      stage1_normals_(new NormalCloud()),
      stage2_cloud_(new Cloud()),
      stage3_cloud_(new Cloud()),
      stage4_cloud_(new Cloud()),
      dummy_cloud_(new Cloud()),
      stage2_normals_(new NormalCloud()),
      stage3_labels_(new LabelCloud()),
      stage4_labels_(new LabelCloud()),
      stage5_labels_(new LabelCloud()),
      stage3_occluding_cloud_(new Cloud()),
      vis_cloud_(new Cloud()),
      vis_labels_(new LabelCloud()),
      vis_occluding_cloud_(new Cloud()),
      vis_normals_(new NormalCloud()),
      updated_data_(false),
      updated_cloud_(false),
      euclidean_cluster_comparator_(
          new pcl::EuclideanClusterComparator<PointT, pcl::Normal,
                                              pcl::Label>()),
      debug_(true),
      timing_(false) {
  if (debug_) printf("Start!\n");

  // debug test
  CloudPtr test1(new Cloud());
  printf("test1 refcount: %zu\n", test1.use_count());
  CloudPtr test2(new Cloud());
  test2 = test1;
  printf("test1 refcount after making test2: %zu\n", test1.use_count());
  test1 = CloudPtr(new Cloud());
  printf("test1 refcount after making new test1: %zu\n", test1.use_count());
  // test

  // Set up Normal Estimation
  // ne.setNormalEstimationMethod (ne.SIMPLE_3D_GRADIENT);
  ne_.setNormalEstimationMethod(ne_.COVARIANCE_MATRIX);
  ne_.setMaxDepthChangeFactor(0.02f);
  ne_.setNormalSmoothingSize(20.0f);

  // Set up plane segmentation
  mps_.setMinInliers(10000);
  mps_.setAngularThreshold(pcl::deg2rad(2.0));  // 2.0
  mps_.setDistanceThreshold(0.02);              // 0.03
  mps_.setProjectPoints(true);
  mps_.setRemoveDuplicatePoints(true);
  pcl::PlaneCoefficientComparator<pcl::PointXYZRGBA, pcl::Normal>::Ptr
      plane_compare(new pcl::PlaneCoefficientComparator<pcl::PointXYZRGBA,
                                                        pcl::Normal>());
  plane_compare->setAngularThreshold(pcl::deg2rad(2.0));  // 3.0
  plane_compare->setDistanceThreshold(0.01, true);        // 0.02, true
  mps_.setComparator(plane_compare);

  pcl::PlaneRefinementComparator<pcl::PointXYZRGBA, pcl::Normal,
                                 pcl::Label>::Ptr
      refine_compare(
          new pcl::PlaneRefinementComparator<pcl::PointXYZRGBA, pcl::Normal,
                                             pcl::Label>());
  refine_compare->setDistanceThreshold(0.01, true);  // 0.01, true//0.0025
  mps_.setRefinementComparator(refine_compare);

  // Set up edge detection
  oed_.setDepthDisconThreshold(0.04f);
  oed_.setMaxSearchNeighbors(100);
  oed_.setEdgeType(oed_.EDGELABEL_NAN_BOUNDARY | oed_.EDGELABEL_OCCLUDING |
                   oed_.EDGELABEL_OCCLUDED);
  // oed.setEdgeType (oed.EDGELABEL_RGB_CANNY);

  // Set up Grabber
  boost::function<void(const CloudConstPtr&)> f =
      boost::bind(&OrganizedFeatureExtraction::CloudCallback, this, _1);
  boost::signals2::connection c = grabber_.registerCallback(f);
  grabber_.start();

  // Set up output
  if (timing_) {
    ne_times_file_.open("/home/atrevor/Desktop/ne_times.txt");
    mps_times_file_.open("/home/atrevor/Desktop/mps_times.txt");
  }

  PCL_INFO("Starting process thread\n");
  process_thread_ =
      boost::thread(&OrganizedFeatureExtraction::ProcessFrame, this);
}

// Get latest cloud from the sensor
template <typename PointT>
void OrganizedFeatureExtraction<PointT>::CloudCallback(
    const CloudConstPtr& cloud) {
  // Store cloud
  boost::mutex::scoped_lock(cloud_mutex);
  {
    // boost::lock_guard<boost::mutex> lock (cloud_mutex);
    // std::cout << "OrganizedFeatureExtraction: Cloud stamp: " <<
    // cloud->header.stamp << std::endl;
    FPS_CALC("cloud_callback");
    prev_sensor_cloud_ = cloud;
    updated_cloud_ = true;
  }
  // updated_cond_.notify_one ();
}

// Get latest cloud from the sensor
// template <typename PointT> void
// OrganizedFeatureExtraction<PointT>::cloudCallbackProcess (const
// CloudConstPtr& cloud)
// {
//   // Store cloud
//   boost::mutex::scoped_lock (cloud_mutex);
//   //std::cout << "OrganizedFeatureExtraction: Cloud stamp: " <<
//   cloud->header.stamp << std::endl; FPS_CALC ("cloud_callback");
//   //prev_sensor_cloud_ = cloud;
//   //stage1_cloud_ = prev_sensor_cloud_;
//   stage1_cloud_ = cloud;

//   if (true)
//   {

//     double start = pcl::getTime ();
//     ne.setInputCloud (stage1_cloud_);
//     boost::thread ne_thread (&OrganizedFeatureExtraction::computeNormals,
//     this);

//     mps.setInputNormals (stage2_normals_);
//     mps.setInputCloud (stage2_cloud_);
//     boost::thread mps_thread (&OrganizedFeatureExtraction::computePlanes,
//     this);

//     ne_thread.join ();
//     mps_thread.join ();
//     std::cout << "time: " << double(pcl::getTime () - start) << std::endl;

//     //if (label_cloud_callback_)
//     //  label_cloud_callback_ (stage2_cloud_, stage3_labels_);

//     {
//       // boost::mutex::scoped_lock lock (vis_mutex);
//       // // Store the latest results for the visualizer
//       // vis_cloud_ = stage2_cloud_;
//       // vis_labels_ = stage3_labels_;
//       // vis_normals_ = stage2_normals_;
//       // vis_regions_ = stage3_regions_;
//       // vis_occluding_cloud_ = stage3_occluding_cloud_;
//       // Update completed stage1 to stage2
//       stage2_cloud_ = stage1_cloud_;
//       stage2_normals_ = stage1_normals_;
//       //updated_data_ = true;
//     }
//   }

//   //updated_cloud_ = true;
// }

template <typename PointT>
void OrganizedFeatureExtraction<PointT>::ProcessFrame() {
  while (true) {
    // CloudConstPtr cloud;
    // Get the latest cloud from the sensor
    bool should_process = false;
    if (cloud_mutex_.try_lock()) {
      //{
      // boost::lock_guard<boost::mutex> lock (cloud_mutex);
      // boost::mutex::scoped_lock (cloud_mutex);
      should_process = updated_cloud_;
      if (should_process) stage1_cloud_ = prev_sensor_cloud_;
      updated_cloud_ = false;
      cloud_mutex_.unlock();
    }

    // condition var test
    // {
    //   boost::unique_lock<boost::mutex> lock (cloud_mutex);
    //   while (!updated_cloud_)
    //   {
    //     updated_cond_.wait (lock);
    //   }
    //   stage1_cloud_ = prev_sensor_cloud_;
    //   should_process = true;
    //   updated_cloud_ = false;
    // }
    // end codition var test

    if (!should_process) {
      boost::this_thread::sleep(boost::posix_time::milliseconds(1));
      // printf ("no new data\n");
    } else {
      FPS_CALC("processing");
      // printf ("new data: %d\n", stage1_cloud_->points.size ());

      // Kick off a thread to estimate surface normals for this frame
      // printf ("Stage1 cloud has: %d\n", stage1_cloud_->points.size ());

      ne_.setInputCloud(stage1_cloud_);
      // ne.compute (*stage1_normals_);
      // boost::thread ne_thread (&pcl::IntegralImageNormalEstimation<PointT,
      // pcl::Normal>::compute, &ne, *stage1_normals_); ne_thread.join ();
      double ne_thread_spawn = pcl::getTime();
      boost::thread ne_thread(
          boost::bind(&OrganizedFeatureExtraction::ComputeNormals, this));
      // ne_thread.join ();

      // Now we extract features for the frame before this, for which we now
      // have normals Extract planes
      mps_.setInputNormals(stage2_normals_);
      mps_.setInputCloud(stage2_cloud_);
      // if (stage2_cloud_->points.size () > 0)
      //  mps.segmentAndRefine (regions);
      double mps_thread_spawn = pcl::getTime();
      boost::thread mps_thread(
          boost::bind(&OrganizedFeatureExtraction::ComputePlanes, this));

      // Extract edges
      oed_.setInputCloud(stage2_cloud_);
      double oed_thread_spawn = pcl::getTime();
      boost::thread oed_thread(
          boost::bind(&OrganizedFeatureExtraction::ComputeEdges, this));

      // Compute Euclidean Clusters
      double clust_thread_spawn = pcl::getTime();
      boost::thread clust_thread(
          boost::bind(&OrganizedFeatureExtraction::ComputeClusters, this));

      // Wait for these to all complete
      ne_thread.join();
      if (debug_)
        std::cout << "NE thread joined: "
                  << double(pcl::getTime() - ne_thread_spawn) << std::endl;
      oed_thread.join();
      if (debug_)
        std::cout << "OED thread joined: "
                  << double(pcl::getTime() - oed_thread_spawn) << std::endl;
      mps_thread.join();
      if (debug_)
        std::cout << "MPS thread joined: "
                  << double(pcl::getTime() - mps_thread_spawn) << std::endl;
      clust_thread.join();
      if (debug_)
        std::cout << "Clust thread joined: "
                  << double(pcl::getTime() - clust_thread_spawn) << std::endl;

      // printf ("publishing with %d %d\n", stage2_cloud_->points.size (),
      // labels->points.size ());
      // if (label_cloud_callback_)
      // {
      //   std::cout << "Starting label callback!" << std::endl;
      //   std::cout << "Stage 2 cloud: " << stage2_cloud_->points.size () <<
      //   std::endl; std::cout << "Stage3 labels: " <<
      //   stage3_labels_->points.size () << std::endl; double cb_start =
      //   pcl::getTime (); label_cloud_callback_ (stage2_cloud_,
      //   stage3_labels_); std::cout << "Callback took: " << double
      //   (pcl::getTime () - cb_start) << std::endl;
      // }

      if (cluster_label_cloud_callbacks_.size() > 0) {
        if ((stage4_cloud_->points.size() > 200) &&
            (stage5_labels_->points.size() > 200)) {
          std::cout << "Starting cluster label cloud callback!" << std::endl;
          std::cout << "Stage4 cloud: " << stage4_cloud_->points.size()
                    << std::endl;
          std::cout << "stage5 labels: " << stage5_labels_->points.size()
                    << std::endl;
          for (std::size_t i = 0; i < cluster_label_cloud_callbacks_.size();
               i++)
            cluster_label_cloud_callbacks_[i](stage4_cloud_, stage5_labels_);
        }
      }

      if (cluster_cloud_callbacks_.size() > 0) {
        if (stage5_clusters_.size() > 0) {
          Time timestamp = StampToPtime(stage4_cloud_->header.stamp);
          for (std::size_t i = 0; i < cluster_cloud_callbacks_.size(); i++) {
            cluster_cloud_callbacks_[i](stage5_clusters_, timestamp,
                                        stage5_cluster_indices_);
          }
        }
      }

      // if (region_cloud_callback_)
      // {
      //   double cb_start = pcl::getTime ();
      //   region_cloud_callback_ (stage2_cloud_, stage3_regions_);
      //   std::cout << "Callback took: " << double (pcl::getTime () - cb_start)
      //   << std::endl;
      // }

      if (occluding_edge_callback_) {
        if (stage3_occluding_cloud_->points.size() > 200)
          occluding_edge_callback_(stage3_occluding_cloud_);
      }

      if (planar_region_stamped_callbacks_.size() > 0) {
        std::cout << "Starting planar region stamped callback" << std::endl;
        std::cout << "stage3 regions has: " << stage3_regions_.size()
                  << std::endl;
        Time timestamp = StampToPtime(stage2_cloud_->header.stamp);
        if (stage2_cloud_->points.size() > 200) {
          for (std::size_t i = 0; i < planar_region_stamped_callbacks_.size();
               i++)
            planar_region_stamped_callbacks_[i](stage3_regions_, timestamp);
        }
      }

      // Store result
      // {
      //   // boost::mutex::scoped_lock lock (vis_mutex);
      //   // // Store the latest results for the visualizer
      //   // Update completed stage1 to stage2
      //   boost::lock_guard<boost::mutex> lock (cloud_mutex);
      //   stage4_regions_ = stage3_regions_;
      //   stage4_cloud_ = stage3_cloud_;
      //   stage3_cloud_ = stage2_cloud_;
      //   stage2_cloud_ = stage1_cloud_;
      //   stage2_normals_ = stage1_normals_;
      //   stage4_labels_ = stage3_labels_;
      //   //stage4_labels_.swap (stage3_labels_);
      //   stage4_model_coefficients_ = stage3_model_coefficients_;
      //   stage4_inlier_indices_ = stage3_inlier_indices_;
      //   stage4_label_indices_ = stage3_label_indices_;
      //   stage4_boundary_indices_ =  stage3_boundary_indices_;
      //   stage3_model_coefficients_.clear ();

      //   // updated_data_ = true;
      //   printf ("stage1_cloud use_count: %d\n", stage1_cloud_.use_count ());
      //   printf ("stage2_cloud use_count: %d\n", stage2_cloud_.use_count ());
      //   printf ("stage3_cloud use_count: %d\n", stage3_cloud_.use_count ());
      //   printf ("stage4_cloud use_count: %d\n", stage4_cloud_.use_count ());
      // }

      // alt
      {
        boost::mutex::scoped_lock lock(cloud_mutex_);
        stage4_regions_ = stage3_regions_;
        stage3_regions_.clear();
        std::cout << "stage1 stamp: " << stage1_cloud_->header.stamp
                  << std::endl;
        std::cout << "stage2 stamp: " << stage2_cloud_->header.stamp
                  << std::endl;
        std::cout << "stage3 stamp: " << stage3_cloud_->header.stamp
                  << std::endl;
        stage4_cloud_.swap(stage3_cloud_);
        stage3_cloud_.swap(stage2_cloud_);
        stage2_cloud_.swap(stage1_cloud_);
        std::cout << "stage1 stamp after: " << stage1_cloud_->header.stamp
                  << std::endl;
        std::cout << "stage2 stamp after: " << stage2_cloud_->header.stamp
                  << std::endl;
        std::cout << "stage3 stamp after: " << stage3_cloud_->header.stamp
                  << std::endl;
        std::cout << "stage4 stamp after: " << stage4_cloud_->header.stamp
                  << std::endl;
        stage2_normals_ = stage1_normals_;
        // stage4_labels_.swap (stage3_labels_);
        stage4_labels_ = stage3_labels_;
        stage3_labels_ = LabelCloudPtr(new LabelCloud());
        stage4_model_coefficients_ = stage3_model_coefficients_;
        stage4_inlier_indices_ = stage3_inlier_indices_;
        stage4_label_indices_ = stage3_label_indices_;
        stage4_boundary_indices_ = stage3_boundary_indices_;
      }
    }
  }
}

// Compute the normals
template <typename PointT>
void OrganizedFeatureExtraction<PointT>::ComputeNormals() {
  double start = pcl::getTime();
  ne_.compute(*stage1_normals_);
  double end = pcl::getTime();
  if (timing_) {
    ne_times_file_ << double(end - start) << std::endl;
    std::cout << double(end - start) << std::endl;
  }
  // DUMMY DEBUG
  // ne.setInputCloud (dummy_cloud_);
}

// Compute clusters
template <typename PointT>
void OrganizedFeatureExtraction<PointT>::ComputeClusters() {
  boost::lock_guard<boost::mutex> lock(state_mutex_);

  // printf ("Compute clusters: stage4_cloud_.use_count(): %d\n",
  // stage4_cloud_.use_count ());

  if (stage4_labels_->points.size() == 0 || stage4_cloud_->points.size() == 0)
    return;
  // Segment Objects
  // typename pcl::PointCloud<PointT>::CloudVectorType clusters;
  stage5_clusters_.clear();
  stage5_cluster_indices_.clear();

  stage5_labels_ = LabelCloudPtr(new LabelCloud());
  // pcl::copyPointCloud (*stage4_labels_, *stage5_labels_);

  std::vector<bool> plane_labels;
  plane_labels.resize(stage4_label_indices_.size(), false);

  if (stage4_regions_.size() > 0) {
    for (size_t i = 0; i < stage4_label_indices_.size(); i++) {
      if (stage4_label_indices_[i].indices.size() > 10000) {
        plane_labels[i] = true;
      }
    }

    // for (size_t i = 0; i < stage4_inlier_indices_.size (); i++)
    // {
    //   if (stage4_inlier_indices_[i].indices.size () > 10000)
    //   {
    //     plane_labels[stage4_labels_->points[stage4_inlier_indices_[i].indices[0]
    //     ].label] = true;
    //   }
    // }
  }

  printf("stage4 cloud has: %zu labels has %zu, exclude labels has %zu\n",
         stage4_cloud_->points.size(), stage4_labels_->points.size(),
         plane_labels.size());
  euclidean_cluster_comparator_->setInputCloud(stage4_cloud_);
  euclidean_cluster_comparator_->setLabels(stage4_labels_);
  euclidean_cluster_comparator_->setExcludeLabels(plane_labels);
  euclidean_cluster_comparator_->setDistanceThreshold(0.01f, false);

  pcl::PointCloud<pcl::Label> euclidean_labels;
  std::vector<pcl::PointIndices> euclidean_label_indices;
  pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label>
      euclidean_segmentation(euclidean_cluster_comparator_);
  euclidean_segmentation.setInputCloud(stage4_cloud_);
  printf("Calling segment for euclidean clusters\n");
  euclidean_segmentation.segment(*stage5_labels_, euclidean_label_indices);
  printf("Done with segment for euclidean clusters\n");

  for (size_t i = 0; i < euclidean_label_indices.size(); i++) {
    if (euclidean_label_indices[i].indices.size() > 1000) {
      // pcl::PointCloud<PointT> cluster;
      CloudPtr cluster(new Cloud());
      pcl::copyPointCloud(*stage4_cloud_, euclidean_label_indices[i].indices,
                          *cluster);
      // clusters.push_back (cluster);

      stage5_clusters_.push_back(cluster);
      stage5_cluster_indices_.push_back(euclidean_label_indices[i]);
    }
  }

  PCL_INFO("Got %d euclidean clusters!\n", stage5_clusters_.size());
}

// Compute planes
template <typename PointT>
void OrganizedFeatureExtraction<PointT>::ComputePlanes() {
  // printf ("OrganizedFeature extraction: computePlanes:
  // stage2_cloud_.use_count: %d\n", stage2_cloud_.use_count ());
  if (stage2_cloud_->points.size() == 0) return;

  double init_start = pcl::getTime();
  std::vector<pcl::ModelCoefficients> model_coefficients;
  std::vector<pcl::PointIndices> inlier_indices;
  std::vector<pcl::PointIndices> label_indices;
  std::vector<pcl::PointIndices> boundary_indices;
  stage3_model_coefficients_ = model_coefficients;
  stage3_inlier_indices_ = inlier_indices;
  stage3_label_indices_ = label_indices;
  stage3_boundary_indices_ = boundary_indices;
  stage3_labels_ = LabelCloudPtr(new LabelCloud());
  stage3_regions_.clear();
  std::cout << "MPS init took: " << double(pcl::getTime() - init_start)
            << std::endl;
  std::cout << "Checking  stage3 labels!" << std::endl;
  std::cout << "stage3_labels_ size: " << stage3_labels_->points.size()
            << std::endl;

  double start = pcl::getTime();
  // mps.segment (stage3_regions_);
  // mps.segmentAndRefine (stage3_regions_);
  mps_.segmentAndRefine(stage3_regions_, stage3_model_coefficients_,
                        stage3_inlier_indices_, stage3_labels_,
                        stage3_label_indices_, stage3_boundary_indices_);

  double end = pcl::getTime();
  std::cout << "mps segment and refine took: " << double(end - start)
            << std::endl;
  // char time_str[2048];
  // sprintf (time_str,"%lf\n",double(end - start));
  // printf ("MPS segmentandrefine took: %s\n", time_str);
  if (timing_) {
    mps_times_file_ << double(end - start) << std::endl;
    std::cout << double(end - start) << std::endl;
  }
  printf("Got %zu regions!\n", stage3_regions_.size());
}

// Extract edges
template <typename PointT>
void OrganizedFeatureExtraction<PointT>::ComputeEdges() {
  if (stage2_cloud_->points.size() == 0) return;

  double edge_start = pcl::getTime();
  pcl::PointCloud<pcl::Label> labels;
  std::vector<pcl::PointIndices> label_indices;
  oed_.compute(labels, label_indices);
  double edge_end = pcl::getTime();
  std::cout << "edges took: " << double(edge_end - edge_start) << std::endl;
  stage3_occluding_cloud_ = CloudPtr(new Cloud());
  pcl::copyPointCloud(*stage2_cloud_, label_indices[1],
                      *stage3_occluding_cloud_);
}

template <typename PointT>
bool OrganizedFeatureExtraction<PointT>::Ready() {
  boost::mutex::scoped_lock lock(cloud_mutex_);
  return true;
}

template <typename PointT>
void OrganizedFeatureExtraction<PointT>::Spin() {
  spin_thread_ =
      boost::thread(&OrganizedFeatureExtraction<PointT>::SpinThread, this);
}

/** \brief Run the visualizer
 *
 */
template <typename PointT>
void OrganizedFeatureExtraction<PointT>::SpinThread() {
  while (true) {
    // viewer_->spinOnce (100);

    boost::this_thread::sleep(boost::posix_time::milliseconds(10));

    CloudConstPtr cloud(new Cloud());
    LabelCloudConstPtr labels(new LabelCloud());
    CloudConstPtr occluding_cloud(new Cloud());
    NormalCloudConstPtr normals(new NormalCloud());
    std::vector<pcl::PlanarRegion<PointT>,
                Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >
        regions;
    bool should_update = false;

    if (vis_mutex_.try_lock()) {
      vis_cloud_.swap(cloud);
      vis_labels_.swap(labels);
      vis_occluding_cloud_.swap(occluding_cloud);
      vis_normals_.swap(normals);
      regions = vis_regions_;

      vis_mutex_.unlock();
    }

    if (cloud) {
      FPS_CALC("visualization1");
      // planar_region_callback_ (regions);
      if (updated_data_) {
        FPS_CALC("visualization2");

        if (label_cloud_callback_) {
          double start = pcl::getTime();
          if (cloud->points.size() > 200 && labels->points.size() > 200)
            label_cloud_callback_(cloud, labels);
          std::cout << "Callback took: " << double(pcl::getTime() - start)
                    << std::endl;
        }

        if (planar_region_callback_) {
          if (cloud->points.size() > 200) planar_region_callback_(regions);
        }

        if (planar_region_stamped_callback_) {
          Time timestamp = StampToPtime(cloud->header.stamp);
          if (cloud->points.size() > 200)
            planar_region_stamped_callback_(regions, timestamp);
        }

        // if (occluding_cloud->points.size () > 200)
        // {
        //   printf ("CALLING OCCLUDING EDGE CALLBACK with %d points!\n",
        //   occluding_cloud->points.size ()); std::cout << " OCCLUDING CLOUD
        //   STAMP: " << cloud->header.stamp << std::endl;
        //   //occluding_edge_callback_ (occluding_cloud);
        // }
        // else
        // {
        //   printf ("TOO FEW POINTS TO CALL OCCLUDING EDGE CALLBACK\n");
        // }

        // if (cloud->points.size () > 200 && labels->points.size () > 200)
        // {
        //   printf ("CALLING LABEL CALLBACK!\n");
        //   label_cloud_callback_ (cloud, labels);
        // }
        // else
        // {
        //   printf ("not enough points!\n");
        // }

        updated_data_ = false;
      }
    }
  }
  process_thread_.join();
}

// TODO: boost::signals2, PCL grabber type
template <typename PointT>
void OrganizedFeatureExtraction<PointT>::SetPlanarRegionCallback(
    boost::function<void(
        std::vector<pcl::PlanarRegion<PointT>,
                    Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >&)>&
        fn) {
  planar_region_callback_ = fn;
}

template <typename PointT>
void OrganizedFeatureExtraction<PointT>::SetPlanarRegionStampedCallback(
    boost::function<
        void(std::vector<pcl::PlanarRegion<PointT>,
                         Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >,
             Time)>& fn) {
  planar_region_stamped_callbacks_.push_back(fn);
  // planar_region_stamped_callback_ = fn;
  printf("planar region stamped callback set!\n");
}

template <typename PointT>
void OrganizedFeatureExtraction<PointT>::SetOccludingEdgeCallback(
    boost::function<void(const CloudConstPtr&)>& fn) {
  occluding_edge_callback_ = fn;
}

template <typename PointT>
void OrganizedFeatureExtraction<PointT>::SetLabelsCallback(
    boost::function<void(const CloudConstPtr&, const LabelCloudConstPtr&)>&
        fn) {
  label_cloud_callback_ = fn;
}

template <typename PointT>
void OrganizedFeatureExtraction<PointT>::SetClusterLabelsCallback(
    boost::function<void(const CloudConstPtr&, const LabelCloudConstPtr&)>&
        fn) {
  cluster_label_cloud_callbacks_.push_back(fn);
}

template <typename PointT>
void OrganizedFeatureExtraction<PointT>::SetRegionCloudCallback(
    boost::function<void(
        const CloudConstPtr&,
        std::vector<pcl::PlanarRegion<PointT>,
                    Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >&)>&
        fn) {
  region_cloud_callback_ = fn;
}

template <typename PointT>
void OrganizedFeatureExtraction<PointT>::SetClusterCloudCallback(
    boost::function<void(std::vector<CloudPtr>, Time,
                         boost::optional<std::vector<pcl::PointIndices> >)>
        fn) {
  cluster_cloud_callbacks_.push_back(fn);
}

}  // namespace omnimapper

// Instantiate
// template class omnimapper::OrganizedFeatureExtraction<pcl::PointXYZ>;
template class omnimapper::OrganizedFeatureExtraction<pcl::PointXYZRGBA>;
