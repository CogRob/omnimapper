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
      debug_(true),
      timing_(false),
      ready_(true) {
  if (debug_) printf("Start!\n");

  // Set up Normal Estimation
  // ne_->setNormalEstimationMethod (ne_->SIMPLE_3D_GRADIENT);
  ne_->setNormalEstimationMethod(ne_->COVARIANCE_MATRIX);
  ne_->setMaxDepthChangeFactor(0.02f);
  ne_->setNormalSmoothingSize(20.0f);

  // Set up plane segmentation
  mps_->setMinInliers(min_plane_inliers_);
  mps_->setAngularThreshold(pcl::deg2rad(2.0));  // 2.0
  mps_->setDistanceThreshold(0.02);              // 0.03
  mps_->setProjectPoints(true);
  mps_->setRemoveDuplicatePoints(false);
  pcl::PlaneCoefficientComparator<pcl::PointXYZRGBA, pcl::Normal>::Ptr
      plane_compare(new pcl::PlaneCoefficientComparator<pcl::PointXYZRGBA,
                                                        pcl::Normal>());
  plane_compare->setAngularThreshold(pcl::deg2rad(2.0));  // 3.0
  plane_compare->setDistanceThreshold(0.01, true);        // 0.02, true
  mps_->setComparator(plane_compare);

  pcl::PlaneRefinementComparator<pcl::PointXYZRGBA, pcl::Normal,
                                 pcl::Label>::Ptr
      refine_compare(
          new pcl::PlaneRefinementComparator<pcl::PointXYZRGBA, pcl::Normal,
                                             pcl::Label>());
  refine_compare->setDistanceThreshold(0.01, true);  // 0.01, true//0.0025
  mps_->setRefinementComparator(refine_compare);

  // Set up edge detection
  oed.setDepthDisconThreshold(0.04f);
  oed.setMaxSearchNeighbors(100);
  oed.setEdgeType(oed.EDGELABEL_NAN_BOUNDARY | oed.EDGELABEL_OCCLUDING |
                  oed.EDGELABEL_OCCLUDED);
  // oed.setEdgeType (oed.EDGELABEL_RGB_CANNY);

  // Set up output
  if (timing_) {
    ne_times_file_.open("/home/atrevor/Desktop/ne_times.txt");
    mps_times_file_.open("/home/atrevor/Desktop/mps_times.txt");
  }

  // PCL_INFO ("Starting process thread\n");
  // process_thread = boost::thread (&OrganizedFeatureExtractionTBB::spin,
  // this);
}

// Get latest cloud from the sensor
template <typename PointT>
void OrganizedFeatureExtractionTBB<PointT>::cloudCallback(
    const CloudConstPtr& cloud) {
  // Store cloud
  boost::mutex::scoped_lock(cloud_mutex);
  {
    // boost::lock_guard<boost::mutex> lock (cloud_mutex);
    // std::cout << "OrganizedFeatureExtractionTBB: Cloud stamp: " <<
    // cloud->header.stamp << std::endl;
    FPS_CALC("cloud_callback");
    prev_sensor_cloud_ = cloud;
    updated_cloud_ = true;
    ready_ = false;
  }
  // updated_cond_.notify_one ();
}

// template <typename PointT> void
// OrganizedFeatureExtractionTBB<PointT>::tbbSpin ()
// {
//   class SpinTask : public tbb::task
//   {
//   private:
//       const OrganizedFeatureExtractionTBB<PointT>* ofe_;
//   public:
//       SpinTask (OrganizedFeatureExtractionTBB<PointT>* ofe) :
//     ofe_ (ofe)
//     {
//     }

//     tbb::task* execute ()
//     {
//       while (true)
//       {
//           ofe_.spinOnce ();
//           //tbb::this_thread::yield ();
//           //tbb::this_thread::sleep_for (5);
//         }
//       }
//   };

//   tbb::empty_task* parent = new (tbb::task::allocate_root () )
//   tbb::empty_task; parent->set_ref_count (2);
// SpinTask* s = new(parent->allocate_child () ) SpinTask (this);
//   parent->enqueue (*s);
// }

template <typename PointT>
bool OrganizedFeatureExtractionTBB<PointT>::ready() {
  boost::mutex::scoped_lock lock(cloud_mutex);
  return (ready_);
}

template <typename PointT>
void OrganizedFeatureExtractionTBB<PointT>::spin() {
  spin_thread =
      boost::thread(&OrganizedFeatureExtractionTBB<PointT>::spinThread, this);
}

template <typename PointT>
void OrganizedFeatureExtractionTBB<PointT>::spinThread() {
  while (true) {
    boost::this_thread::sleep(boost::posix_time::milliseconds(5));
    spinOnce();
  }
}

template <typename PointT>
void OrganizedFeatureExtractionTBB<PointT>::spinOnce() {
  FPS_CALC("spin");

  double frame_start = pcl::getTime();
  // Get latest cloud
  input_cloud_ = boost::none;
  if (cloud_mutex.try_lock()) {
    if (updated_cloud_) {
      input_cloud_ = prev_sensor_cloud_;
      updated_cloud_ = false;
      ready_ = false;
    }
    cloud_mutex.unlock();
  }

  // Normal Estimation && MPS && Clustering
  tbb::task_group group;
  group.run([&] { computeNormals(); });
  group.run([&] { computePlanes(); });
  group.run([&] { computeClusters(); });
  group.run([&] { computeEdges(); });
  group.run([&] { publish(); });
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

  // mps_input_cloud_ = ne_input_cloud_;
  // ne_input_cloud_ = boost::none;
  // ne_output_normals_ = boost::none;
  // stage4_regions_ = mps_output_regions_;
  //   stage4_cloud_ = mps_output_cloud_;
  //   mps_output_cloud_ = stage2_cloud_;
  //   stage2_cloud_ = stage1_cloud_;
  //   mps_input_normals_ = stage1_normals_;
  //   stage4_labels_ = mps_output_labels_;
  double frame_end = pcl::getTime();
  std::cout << "Frame took: " << double(frame_end - frame_start) << std::endl;
}

template <typename PointT>
void OrganizedFeatureExtractionTBB<PointT>::publish() {
  std::cout << "Publishing..." << std::endl;
  // Publish plane Labels
  if (plane_label_cloud_callback_) {
    if (clust_input_cloud_ && clust_input_labels_) {
      plane_label_cloud_callback_(*clust_input_cloud_, *clust_input_labels_);
    }
  }

  // Publish plane regions
  if (planar_region_stamped_callbacks_.size() > 0) {
    if (pub_mps_regions_) {
      Time timestamp = stamp2ptime((*clust_input_cloud_)->header.stamp);
      for (std::size_t i = 0; i < planar_region_stamped_callbacks_.size(); i++)
        planar_region_stamped_callbacks_[i](*pub_mps_regions_, timestamp);
    }
  }

  // Publish Cluster Labels
  if (cluster_label_cloud_callbacks_.size() > 0) {
    if (pub_cluster_cloud_ && pub_cluster_labels_) {
      for (std::size_t i = 0; i < cluster_label_cloud_callbacks_.size(); i++)
        cluster_label_cloud_callbacks_[i](*pub_cluster_cloud_,
                                          *pub_cluster_labels_);
    }
  }

  // Publish Cluster Clouds
  if (cluster_cloud_callbacks_.size() > 0) {
    std::cout << "Have cluster cloud callbacks!" << std::endl;
    if (pub_cluster_cloud_) {
      Time timestamp = stamp2ptime((*pub_cluster_cloud_)->header.stamp);
      for (std::size_t i = 0; i < cluster_cloud_callbacks_.size(); i++) {
        std::cout << "Publishing cluster clouds!\n" << std::endl;
        cluster_cloud_callbacks_[i](*pub_clusters_, timestamp,
                                    *pub_cluster_indices_);
      }
    }
  }

  // Publish Occluding Edges
  if (occluding_edge_callback_) {
    std::cout << "Have occ edge callbacks!" << std::endl;
    if (pub_occluding_edge_cloud_) {
      std::cout << "publishing occ edges!" << std::endl;
      occluding_edge_callback_(*pub_occluding_edge_cloud_);
    }
  }
}

/*
    template <typename PointT> void
    OrganizedFeatureExtractionTBB<PointT>::processFrame ()
    {
      while (true)
      {
        //CloudConstPtr cloud;
        // Get the latest cloud from the sensor
        bool should_process = false;
        if (cloud_mutex.try_lock ()){
        //{
          //boost::lock_guard<boost::mutex> lock (cloud_mutex);
          //boost::mutex::scoped_lock (cloud_mutex);
          should_process = updated_cloud_;
          if (should_process)
            stage1_cloud_ = prev_sensor_cloud_;
          updated_cloud_ = false;
          cloud_mutex.unlock ();
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

        if (!should_process)
        {
          boost::this_thread::sleep (boost::posix_time::milliseconds (1));
          //printf ("no new data\n");
        }
        else
        {
          FPS_CALC ("processing");
          //printf ("new data: %d\n", stage1_cloud_->points.size ());

          // Kick off a thread to estimate surface normals for this frame
          //printf ("Stage1 cloud has: %d\n", stage1_cloud_->points.size ());

          ne.setInputCloud (stage1_cloud_);
          //ne.compute (*stage1_normals_);
          //boost::thread ne_thread (&pcl::IntegralImageNormalEstimation<PointT,
   pcl::Normal>::compute, &ne, *stage1_normals_);
          //ne_thread.join ();
          double ne_thread_spawn = pcl::getTime ();
          boost::thread ne_thread
   (&OrganizedFeatureExtractionTBB::computeNormals, boost::ref (this));
          //ne_thread.join ();

          // Now we extract features for the frame before this, for which we now
   have normals
          // Extract planes
          mps.setInputNormals (stage2_normals_);
          mps.setInputCloud (stage2_cloud_);
          //if (stage2_cloud_->points.size () > 0)
          //  mps.segmentAndRefine (regions);
          double mps_thread_spawn = pcl::getTime ();
          boost::thread mps_thread
   (&OrganizedFeatureExtractionTBB::computePlanes, boost::ref (this));

          // Extract edges
          oed.setInputCloud (stage2_cloud_);
          double oed_thread_spawn = pcl::getTime ();
          boost::thread oed_thread
   (&OrganizedFeatureExtractionTBB::computeEdges, boost::ref (this));

          // Compute Euclidean Clusters
          double clust_thread_spawn = pcl::getTime ();
          boost::thread clust_thread
   (&OrganizedFeatureExtractionTBB::computeClusters, boost::ref (this));

          // Wait for these to all complete
          ne_thread.join ();
          if (debug_)
            std::cout << "NE thread joined: " << double(pcl::getTime () -
   ne_thread_spawn) << std::endl; oed_thread.join (); if (debug_) std::cout <<
   "OED thread joined: " << double(pcl::getTime () - oed_thread_spawn) <<
   std::endl; mps_thread.join (); if (debug_) std::cout << "MPS thread joined: "
   << double(pcl::getTime () - mps_thread_spawn) << std::endl; clust_thread.join
   (); if (debug_) std::cout << "Clust thread joined: " << double (pcl::getTime
   () - clust_thread_spawn) << std::endl;

          //printf ("publishing with %d %d\n", stage2_cloud_->points.size (),
   labels->points.size ());
          // if (label_cloud_callback_)
          // {
          //   std::cout << "Starting label callback!" << std::endl;
          //   std::cout << "Stage 2 cloud: " << stage2_cloud_->points.size ()
   << std::endl;
          //   std::cout << "Stage3 labels: " << mps_output_labels_->points.size
   () << std::endl;
          //   double cb_start = pcl::getTime ();
          //   label_cloud_callback_ (stage2_cloud_, mps_output_labels_);
          //   std::cout << "Callback took: " << double (pcl::getTime () -
   cb_start) << std::endl;
          // }

          if (cluster_label_cloud_callbacks_.size() >0)
          {
            if ((stage4_cloud_->points.size () > 200) &&
   (stage5_labels_->points.size () > 200))
            {
              std::cout << "Starting cluster label cloud callback!" <<
   std::endl; std::cout << "Stage4 cloud: " << stage4_cloud_->points.size () <<
   std::endl; std::cout << "stage5 labels: " << stage5_labels_->points.size ()
   << std::endl; for(int i=0; i< cluster_label_cloud_callbacks_.size(); i++)
               cluster_label_cloud_callbacks_[i] (stage4_cloud_,
   stage5_labels_);
            }
          }

          if (cluster_cloud_callbacks_.size () > 0)
          {
            if (stage5_clusters_.size () > 0)
            {
              Time timestamp = stamp2ptime (stage4_cloud_->header.stamp);
              for (int i = 0; i < cluster_cloud_callbacks_.size (); i++)
              {
                cluster_cloud_callbacks_[i] (stage5_clusters_, timestamp,
   stage5_cluster_indices_);
              }
            }
          }

          // if (region_cloud_callback_)
          // {
          //   double cb_start = pcl::getTime ();
          //   region_cloud_callback_ (stage2_cloud_, mps_output_regions_);
          //   std::cout << "Callback took: " << double (pcl::getTime () -
   cb_start) << std::endl;
          // }

          if (occluding_edge_callback_)
          {
            if (mps_output_occluding_cloud_->points.size () > 200)
              occluding_edge_callback_ (mps_output_occluding_cloud_);
          }

          if (planar_region_stamped_callbacks_.size () > 0)
          {
            std::cout << "Starting planar region stamped callback" << std::endl;
            std::cout << "stage3 regions has: " << mps_output_regions_.size ()
   << std::endl; Time timestamp = stamp2ptime (stage2_cloud_->header.stamp); if
   (stage2_cloud_->points.size () > 200)
            {
              for (int i = 0; i < planar_region_stamped_callbacks_.size (); i++)
                planar_region_stamped_callbacks_[i] (mps_output_regions_,
   timestamp);
            }
          }


          // Store result
          // {
          //   // boost::mutex::scoped_lock lock (vis_mutex);
          //   // // Store the latest results for the visualizer
          //   // Update completed stage1 to stage2
          //   boost::lock_guard<boost::mutex> lock (cloud_mutex);
          //   stage4_regions_ = mps_output_regions_;
          //   stage4_cloud_ = mps_output_cloud_;
          //   mps_output_cloud_ = stage2_cloud_;
          //   stage2_cloud_ = stage1_cloud_;
          //   stage2_normals_ = stage1_normals_;
          //   stage4_labels_ = mps_output_labels_;
          //   //stage4_labels_.swap (mps_output_labels_);
          //   stage4_model_coefficients_ = mps_output_model_coefficients_;
          //   stage4_inlier_indices_ = mps_output_inlier_indices_;
          //   stage4_label_indices_ = mps_output_label_indices_;
          //   stage4_boundary_indices_ =  mps_output_boundary_indices_;
          //   mps_output_model_coefficients_.clear ();

          //   // updated_data_ = true;
          //   printf ("stage1_cloud use_count: %d\n", stage1_cloud_.use_count
   ());
          //   printf ("stage2_cloud use_count: %d\n", stage2_cloud_.use_count
   ());
          //   printf ("mps_output_cloud use_count: %d\n",
   mps_output_cloud_.use_count ());
          //   printf ("stage4_cloud use_count: %d\n", stage4_cloud_.use_count
   ());
          // }

          // alt
          {
            boost::mutex::scoped_lock lock (cloud_mutex);
            stage4_regions_ = mps_output_regions_;
            mps_output_regions_.clear ();
            std::cout << "stage1 stamp: " << stage1_cloud_->header.stamp <<
   std::endl; std::cout << "stage2 stamp: " << stage2_cloud_->header.stamp <<
   std::endl; std::cout << "stage3 stamp: " << mps_output_cloud_->header.stamp
   << std::endl; stage4_cloud_.swap (mps_output_cloud_); mps_output_cloud_.swap
   (stage2_cloud_); stage2_cloud_.swap (stage1_cloud_); std::cout << "stage1
   stamp after: " << stage1_cloud_->header.stamp << std::endl; std::cout <<
   "stage2 stamp after: " << stage2_cloud_->header.stamp << std::endl; std::cout
   << "stage3 stamp after: " << mps_output_cloud_->header.stamp << std::endl;
            std::cout << "stage4 stamp after: " << stage4_cloud_->header.stamp
   << std::endl; stage2_normals_ = stage1_normals_;
            //stage4_labels_.swap (mps_output_labels_);
            stage4_labels_ = mps_output_labels_;
            mps_output_labels_ = LabelCloudPtr (new LabelCloud ());
            stage4_model_coefficients_ = mps_output_model_coefficients_;
            stage4_inlier_indices_ = mps_output_inlier_indices_;
            stage4_label_indices_ = mps_output_label_indices_;
            stage4_boundary_indices_ =  mps_output_boundary_indices_;
          }

        }

      }

    }
*/

// Compute the normals
template <typename PointT>
void OrganizedFeatureExtractionTBB<PointT>::computeNormals() {
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
    std::cout << double(end - start) << std::endl;
  }
  std::cout << "NE took : " << double(end - start) << std::endl;
  return;
}

// Compute clusters
template <typename PointT>
void OrganizedFeatureExtractionTBB<PointT>::computeClusters() {
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

  // printf ("stage4 cloud has: %d labels has %d, exclude labels has %d\n",
  // stage4_cloud_->points.size (), stage4_labels_->points.size (),
  // plane_labels.size ());
  euclidean_cluster_comparator_->setInputCloud(*clust_input_cloud_);
  euclidean_cluster_comparator_->setLabels(*clust_input_labels_);
  euclidean_cluster_comparator_->setExcludeLabels(plane_labels);
  euclidean_cluster_comparator_->setDistanceThreshold(0.01f, false);

  pcl::PointCloud<pcl::Label> euclidean_labels;
  std::vector<pcl::PointIndices> euclidean_label_indices;
  pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label>
      euclidean_segmentation(euclidean_cluster_comparator_);
  euclidean_segmentation.setInputCloud(*clust_input_cloud_);
  printf("Calling segment for euclidean clusters\n");
  euclidean_segmentation.segment(*(*clust_output_labels_),
                                 euclidean_label_indices);
  printf("Done with segment for euclidean clusters\n");

  for (size_t i = 0; i < euclidean_label_indices.size(); i++) {
    if (euclidean_label_indices[i].indices.size() > 1000) {
      // pcl::PointCloud<PointT> cluster;
      CloudPtr cluster(new Cloud());
      pcl::copyPointCloud(*(*clust_input_cloud_),
                          euclidean_label_indices[i].indices, *cluster);

      clust_output_clusters_->push_back(cluster);
      clust_output_cluster_indices_->push_back(euclidean_label_indices[i]);
    }
  }

  PCL_INFO("Got %d euclidean clusters!\n", clust_output_clusters_->size());
}

// Compute planes
template <typename PointT>
void OrganizedFeatureExtractionTBB<PointT>::computePlanes() {
  // std::cout << "in compute planes" << std::endl;

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

  // std::cout << "MPS init took: " << double (pcl::getTime () - init_start) <<
  // std::endl; std::cout << "Checking  stage3 labels!" << std::endl;
  // std::cout << "mps_output_labels_ size: " <<
  // (*mps_output_labels_)->points.size () << std::endl;

  double start = pcl::getTime();
  // mps.segment (mps_output_regions_);
  // mps_->segmentAndRefine (*mps_output_regions_);
  std::cout << "refine" << std::endl;

  mps_->segmentAndRefine(
      (*mps_output_regions_), (*mps_output_model_coefficients_),
      (*mps_output_inlier_indices_), (*mps_output_labels_),
      (*mps_output_label_indices_), (*mps_output_boundary_indices_));

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
  if (mps_output_regions_)
    printf("Got %zu regions!\n", mps_output_regions_->size());
}

// Extract edges
template <typename PointT>
void OrganizedFeatureExtractionTBB<PointT>::computeEdges() {
  if (!input_cloud_) {
    std::cout << "edges returning early" << std::endl;
    oed_output_occluding_edge_cloud_ = boost::none;
    return;
  }

  double edge_start = pcl::getTime();
  pcl::PointCloud<pcl::Label> labels;
  std::vector<pcl::PointIndices> label_indices;
  oed.setInputCloud(*input_cloud_);

  oed.compute(labels, label_indices);
  double edge_end = pcl::getTime();
  std::cout << "edges took: " << double(edge_end - edge_start) << std::endl;
  // oed_output_occluding_edge_cloud_ = CloudPtr(new Cloud ());
  CloudPtr edge_cloud(new Cloud());
  pcl::copyPointCloud(*(*input_cloud_), label_indices[1], *edge_cloud);
  oed_output_occluding_edge_cloud_ = edge_cloud;
}

template <typename PointT>
void OrganizedFeatureExtractionTBB<PointT>::setPlanarRegionCallback(
    boost::function<void(
        std::vector<pcl::PlanarRegion<PointT>,
                    Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >&)>&
        fn) {
  planar_region_callback_ = fn;
}

template <typename PointT>
void OrganizedFeatureExtractionTBB<PointT>::setPlanarRegionStampedCallback(
    boost::function<
        void(std::vector<pcl::PlanarRegion<PointT>,
                         Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >,
             Time)>& fn) {
  planar_region_stamped_callbacks_.push_back(fn);
  // planar_region_stamped_callback_ = fn;
  printf("planar region stamped callback set!\n");
}

template <typename PointT>
void OrganizedFeatureExtractionTBB<PointT>::setOccludingEdgeCallback(
    boost::function<void(const CloudConstPtr&)>& fn) {
  occluding_edge_callback_ = fn;
}

template <typename PointT>
void OrganizedFeatureExtractionTBB<PointT>::setPlaneLabelsCallback(
    boost::function<void(const CloudConstPtr&, const LabelCloudConstPtr&)>&
        fn) {
  plane_label_cloud_callback_ = fn;
}

template <typename PointT>
void OrganizedFeatureExtractionTBB<PointT>::setClusterLabelsCallback(
    boost::function<void(const CloudConstPtr&, const LabelCloudConstPtr&)>&
        fn) {
  cluster_label_cloud_callbacks_.push_back(fn);
}

template <typename PointT>
void OrganizedFeatureExtractionTBB<PointT>::setRegionCloudCallback(
    boost::function<void(
        const CloudConstPtr&,
        std::vector<pcl::PlanarRegion<PointT>,
                    Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >&)>&
        fn) {
  region_cloud_callback_ = fn;
}

template <typename PointT>
void OrganizedFeatureExtractionTBB<PointT>::setClusterCloudCallback(
    boost::function<void(std::vector<CloudPtr>, Time,
                         boost::optional<std::vector<pcl::PointIndices> >)>
        fn) {
  cluster_cloud_callbacks_.push_back(fn);
}

}  // namespace omnimapper

// Instantiate
// template class omnimapper::OrganizedFeatureExtractionTBB<pcl::PointXYZ>;
template class omnimapper::OrganizedFeatureExtractionTBB<pcl::PointXYZRGBA>;
