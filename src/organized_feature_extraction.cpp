#include <omnimapper/organized_feature_extraction.h>

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

namespace omnimapper
{
  template <typename PointT>
  OrganizedFeatureExtraction<PointT>::OrganizedFeatureExtraction (pcl::Grabber& grabber) 
    : grabber_ (grabber),
      prev_sensor_cloud_ (new Cloud ()),
      stage1_cloud_ (new Cloud ()),
      stage1_normals_ (new NormalCloud ()),
      stage2_cloud_ (new Cloud ()),
      stage2_normals_ (new NormalCloud ()),
      stage3_labels_ (new LabelCloud ()),
      stage3_occluding_cloud_ (new Cloud ()),
      vis_cloud_ (new Cloud ()),
      vis_labels_ (new LabelCloud ()),
      vis_occluding_cloud_ (new Cloud ()),
      vis_normals_ (new NormalCloud ()),
      updated_data_ (false),
      updated_cloud_ (false),
      debug_ (true),
      timing_ (false)
    {
      if (debug_)
        printf ("Start!\n");

      // Set up Normal Estimation
      ne.setNormalEstimationMethod (ne.SIMPLE_3D_GRADIENT);
      //ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
      ne.setMaxDepthChangeFactor (0.02f);
      ne.setNormalSmoothingSize (20.0f);

      // Set up plane segmentation
      mps.setMinInliers (10000);
      mps.setAngularThreshold (pcl::deg2rad (3.0));
      mps.setDistanceThreshold (0.01);

      // Set up edge detection
      oed.setDepthDisconThreshold (0.04f);
      oed.setMaxSearchNeighbors (100);
      oed.setEdgeType (oed.EDGELABEL_NAN_BOUNDARY | oed.EDGELABEL_OCCLUDING | oed.EDGELABEL_OCCLUDED);
      
      // Set up Grabber
      boost::function<void (const CloudConstPtr&)> f = boost::bind (&OrganizedFeatureExtraction::cloudCallback, this, _1);
      boost::signals2::connection c = grabber_.registerCallback (f);
      grabber_.start ();

      // Set up output
      if (timing_)
      {
        ne_times_file_.open ("/home/atrevor/Desktop/ne_times.txt");
        mps_times_file_.open ("/home/atrevor/Desktop/mps_times.txt");
      }

      PCL_INFO ("Starting process thread\n");
      process_thread = boost::thread (&OrganizedFeatureExtraction::processFrame, this);
    }
    
    // Get latest cloud from the sensor
    template <typename PointT> void 
    OrganizedFeatureExtraction<PointT>::cloudCallback (const CloudConstPtr& cloud)
    {
      // Store cloud
      boost::mutex::scoped_lock (cloud_mutex);
      //std::cout << "OrganizedFeatureExtraction: Cloud stamp: " << cloud->header.stamp << std::endl;
      FPS_CALC ("cloud_callback");
      prev_sensor_cloud_ = cloud;
      updated_cloud_ = true;
    }

    // Get latest cloud from the sensor
    template <typename PointT> void 
    OrganizedFeatureExtraction<PointT>::cloudCallbackProcess (const CloudConstPtr& cloud)
    {
      // Store cloud
      boost::mutex::scoped_lock (cloud_mutex);
      //std::cout << "OrganizedFeatureExtraction: Cloud stamp: " << cloud->header.stamp << std::endl;
      FPS_CALC ("cloud_callback");
      //prev_sensor_cloud_ = cloud;
      //stage1_cloud_ = prev_sensor_cloud_;
      stage1_cloud_ = cloud;

      if (true)
      {
        
        double start = pcl::getTime ();
        ne.setInputCloud (stage1_cloud_);
        boost::thread ne_thread (&OrganizedFeatureExtraction::computeNormals, this);
        
        mps.setInputNormals (stage2_normals_);
        mps.setInputCloud (stage2_cloud_);
        boost::thread mps_thread (&OrganizedFeatureExtraction::computePlanes, this);
        
        ne_thread.join ();
        mps_thread.join ();
        std::cout << "time: " << double(pcl::getTime () - start) << std::endl;
        
        
        //if (label_cloud_callback_)
        //  label_cloud_callback_ (stage2_cloud_, stage3_labels_);
        
        {
          // boost::mutex::scoped_lock lock (vis_mutex);
          // // Store the latest results for the visualizer
          // vis_cloud_ = stage2_cloud_;
          // vis_labels_ = stage3_labels_;
          // vis_normals_ = stage2_normals_;
          // vis_regions_ = stage3_regions_;
          // vis_occluding_cloud_ = stage3_occluding_cloud_;
          // Update completed stage1 to stage2
          stage2_cloud_ = stage1_cloud_;
          stage2_normals_ = stage1_normals_;
          //updated_data_ = true;
        }
      }
      
      //updated_cloud_ = true;
    }

    template <typename PointT> void
    OrganizedFeatureExtraction<PointT>::processFrame ()
    {
      while (true)
      {
        //CloudConstPtr cloud;
        // Get the latest cloud from the sensor
        bool should_process = false;
        if (cloud_mutex.try_lock ()){
          //{
          //boost::mutex::scoped_lock (cloud_mutex);
          should_process = updated_cloud_;
          if (should_process)
            stage1_cloud_ = prev_sensor_cloud_;
          updated_cloud_ = false;
          cloud_mutex.unlock ();
        }

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
          //boost::thread ne_thread (&pcl::IntegralImageNormalEstimation<PointT, pcl::Normal>::compute, &ne, *stage1_normals_);
          //ne_thread.join ();
          double ne_thread_spawn = pcl::getTime ();
          boost::thread ne_thread (&OrganizedFeatureExtraction::computeNormals, this);
          //ne_thread.join ();
          
          // Now we extract features for the frame before this, for which we now have normals
          // Extract planes
          // std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
          // std::vector<pcl::ModelCoefficients> model_coefficients;
          // std::vector<pcl::PointIndices> inlier_indices;  
          // pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
          // std::vector<pcl::PointIndices> label_indices;
          // std::vector<pcl::PointIndices> boundary_indices;
          mps.setInputNormals (stage2_normals_);
          mps.setInputCloud (stage2_cloud_);
          //if (stage2_cloud_->points.size () > 0)
          //  mps.segmentAndRefine (regions);
          double mps_thread_spawn = pcl::getTime ();
          boost::thread mps_thread (&OrganizedFeatureExtraction::computePlanes, this);
          
          // Extract edges
          oed.setInputCloud (stage2_cloud_);
          double oed_thread_spawn = pcl::getTime ();
          boost::thread oed_thread (&OrganizedFeatureExtraction::computeEdges, this);
          
          // Wait for these to all complete
          ne_thread.join ();
          if (debug_)
            std::cout << "NE thread joined: " << double(pcl::getTime () - ne_thread_spawn) << std::endl;
          oed_thread.join ();
          if (debug_)
            std::cout << "OED thread joined: " << double(pcl::getTime () - oed_thread_spawn) << std::endl;
          mps_thread.join ();
          if (debug_)
            std::cout << "MPS thread joined: " << double(pcl::getTime () - mps_thread_spawn) << std::endl;
          
          //printf ("publishing with %d %d\n", stage2_cloud_->points.size (), labels->points.size ());
          if (label_cloud_callback_)
          {
            double cb_start = pcl::getTime ();
            label_cloud_callback_ (stage2_cloud_, stage3_labels_);
            std::cout << "Callback took: " << double (pcl::getTime () - cb_start) << std::endl;
          }

          if (region_cloud_callback_)
          {
            double cb_start = pcl::getTime ();
            region_cloud_callback_ (stage2_cloud_, stage3_regions_);
            std::cout << "Callback took: " << double (pcl::getTime () - cb_start) << std::endl;
          }
          
          if (occluding_edge_callback_)
          {
            occluding_edge_callback_ (stage3_occluding_cloud_);
          }
          

          // Store result
          {
            // boost::mutex::scoped_lock lock (vis_mutex);
            // // Store the latest results for the visualizer
            // vis_cloud_ = stage2_cloud_;
            // vis_labels_ = stage3_labels_;
            // vis_normals_ = stage2_normals_;
            // vis_regions_ = stage3_regions_;
            // vis_occluding_cloud_ = stage3_occluding_cloud_;
            // Update completed stage1 to stage2
            stage2_cloud_ = stage1_cloud_;
            stage2_normals_ = stage1_normals_;
            // updated_data_ = true;
          }
        }
        
      }
      
    }

    // Compute the normals
    template <typename PointT> void
    OrganizedFeatureExtraction<PointT>::computeNormals ()
    {
      double start = pcl::getTime ();
      ne.compute (*stage1_normals_);
      double end = pcl::getTime ();
      if (timing_)
      {
        ne_times_file_ << double(end - start) << std::endl;
        std::cout << double(end - start) << std::endl;
      }
    }

  // Compute clusters
  template <typename PointT> void
  OrganizedFeatureExtraction<PointT>::computeClusters ()
  {
  //   if (stage3_labels_->poitns.size () == 0)
  //     return;
  //   //Segment Objects
  //   pcl::PointCloud<PointT>::CloudVectorType clusters;
    
  //   if (regions.size () > 0)
  //   {
  //     std::vector<bool> plane_labels;
  //     plane_labels.resize (label_indices.size (), false);
  //     for (size_t i = 0; i < label_indices.size (); i++)
  //     {
  //       if (label_indices[i].indices.size () > 10000)
  //       {
  //         plane_labels[i] = true;
  //       }
  //     }  
      
  //     euclidean_cluster_comparator_->setInputCloud (cloud);
  //     euclidean_cluster_comparator_->setLabels (labels);
  //     euclidean_cluster_comparator_->setExcludeLabels (plane_labels);
  //     euclidean_cluster_comparator_->setDistanceThreshold (0.01f, false);
      
  //     pcl::PointCloud<pcl::Label> euclidean_labels;
  //     std::vector<pcl::PointIndices> euclidean_label_indices;
  //     pcl::OrganizedConnectedComponentSegmentation<PointT,pcl::Label> euclidean_segmentation (euclidean_cluster_comparator_);
  //     euclidean_segmentation.setInputCloud (cloud);
  //     euclidean_segmentation.segment (euclidean_labels, euclidean_label_indices);
      
  //     for (size_t i = 0; i < euclidean_label_indices.size (); i++)
  //     {
  //       if (euclidean_label_indices[i].indices.size () > 1000)
  //       {
  //         pcl::PointCloud<PointT> cluster;
  //         pcl::copyPointCloud (*cloud,euclidean_label_indices[i].indices,cluster);
  //         clusters.push_back (cluster);
  //       }    
  //     }
      
  //     PCL_INFO ("Got %d euclidean clusters!\n", clusters.size ());
  }
  

    // Compute planes
    template <typename PointT> void
    OrganizedFeatureExtraction<PointT>::computePlanes ()
    {
      if (stage2_cloud_->points.size () == 0)
        return;

      std::vector<pcl::ModelCoefficients> model_coefficients;
      std::vector<pcl::PointIndices> inlier_indices;  
      std::vector<pcl::PointIndices> label_indices;
      std::vector<pcl::PointIndices> boundary_indices;
      stage3_labels_ = LabelCloudPtr(new LabelCloud ());

      double start = pcl::getTime ();
      mps.segment (stage3_regions_);
      //mps.segmentAndRefine (stage3_regions_);
      //mps.segmentAndRefine (stage3_regions_, model_coefficients, inlier_indices, stage3_labels_, label_indices, boundary_indices);
      double end = pcl::getTime ();
      //char time_str[2048];
      //sprintf (time_str,"%lf\n",double(end - start));
      //printf ("MPS took: %s\n", time_str);
      if (timing_)
      {
        mps_times_file_ << double(end - start) << std::endl;
        std::cout << double(end - start) << std::endl;
      }
      printf ("Got %d regions!\n", stage3_regions_.size ());
    }

    // Extract edges
    template <typename PointT> void
    OrganizedFeatureExtraction<PointT>::computeEdges ()
    {
      if (stage2_cloud_->points.size () == 0)
        return;

      pcl::PointCloud<pcl::Label> labels;
      std::vector<pcl::PointIndices> label_indices;
      oed.compute (labels, label_indices);
      pcl::copyPointCloud (*stage2_cloud_, label_indices[1], *stage3_occluding_cloud_);
    }

    /** \brief Run the visualizer
     *
     */
    template <typename PointT> void
    OrganizedFeatureExtraction<PointT>::spin ()
    {
      while (true)
      {
        //viewer_->spinOnce (100);        
       
        boost::this_thread::sleep (boost::posix_time::milliseconds (10));
 
        CloudConstPtr cloud (new Cloud ());
        LabelCloudConstPtr labels (new LabelCloud ());
        CloudConstPtr occluding_cloud (new Cloud ());
        NormalCloudConstPtr normals (new NormalCloud ());
        std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;    
        bool should_update = false;

        if (vis_mutex.try_lock ())
        {
          vis_cloud_.swap (cloud);
          vis_labels_.swap (labels);
          vis_occluding_cloud_.swap (occluding_cloud);
          vis_normals_.swap (normals);
          regions = vis_regions_;
          
          vis_mutex.unlock ();
        }
        
        if (cloud)
        {
          FPS_CALC ("visualization1");
          //planar_region_callback_ (regions);
          if (updated_data_)
          {
            FPS_CALC ("visualization2");
            
            if (label_cloud_callback_)
            {
              double start = pcl::getTime ();
              if (cloud->points.size () > 200 && labels->points.size () > 200)
                label_cloud_callback_ (cloud, labels);
              std::cout << "Callback took: " << double(pcl::getTime () - start) << std::endl;
            }

            if (planar_region_callback_)
            {
              if (cloud->points.size () > 200)
                planar_region_callback_ (regions);

            }
            
            

            // if (occluding_cloud->points.size () > 200)
            // {
            //   printf ("CALLING OCCLUDING EDGE CALLBACK with %d points!\n", occluding_cloud->points.size ());
            //   std::cout << " OCCLUDING CLOUD STAMP: " << cloud->header.stamp << std::endl;
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
      process_thread.join ();
    }

    // TODO: boost::signals2, PCL grabber type
    template <typename PointT> void
    OrganizedFeatureExtraction<PointT>::setPlanarRegionCallback (boost::function<void (std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >&)>& fn)
    {
      planar_region_callback_ = fn;
    }

    template <typename PointT> void
    OrganizedFeatureExtraction<PointT>::setOccludingEdgeCallback (boost::function<void (const CloudConstPtr&)>& fn)
    {
      occluding_edge_callback_ = fn;
    }

  template <typename PointT> void
  OrganizedFeatureExtraction<PointT>::setLabelsCallback (boost::function<void (const CloudConstPtr&, const LabelCloudConstPtr&)>& fn)
  {
    label_cloud_callback_ = fn;
  }

  template <typename PointT> void
  OrganizedFeatureExtraction<PointT>::setRegionCloudCallback (boost::function<void(const CloudConstPtr&, std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >&)>& fn)
  {
    region_cloud_callback_ = fn;
  }
  
    
}

//Instantiate
template class omnimapper::OrganizedFeatureExtraction<pcl::PointXYZRGBA>;
