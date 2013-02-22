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
      stage3_occluding_cloud_ (new Cloud ()),
      vis_cloud_ (new Cloud ()),
      vis_occluding_cloud_ (new Cloud ()),
      vis_normals_ (new NormalCloud ()),
      updated_data_ (false),
      debug_ (false)
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
      
      PCL_INFO ("Starting vis thread\n");
      //vis_thread = boost::thread (&OrganizedFeatureExtraction::visualizeFrame, this);
      PCL_INFO ("Starting process thread\n");
      process_thread = boost::thread (&OrganizedFeatureExtraction::processFrame, this);
    }
    
    // Get latest cloud from the sensor
    template <typename PointT> void 
    OrganizedFeatureExtraction<PointT>::cloudCallback (const CloudConstPtr& cloud)
    {
      // Store cloud
      boost::mutex::scoped_lock (cloud_mutex);
      std::cout << "OrganizedFeatureExtraction: Cloud stamp: " << cloud->header.stamp << std::endl;
      prev_sensor_cloud_ = cloud;
    }

    template <typename PointT> void
    OrganizedFeatureExtraction<PointT>::processFrame ()
    {
      while (true)
      {
        FPS_CALC ("processing");
        //CloudConstPtr cloud;
        // Get the latest cloud from the sensor
        {
          boost::mutex::scoped_lock (cloud_mutex);
          stage1_cloud_ = prev_sensor_cloud_;
        }

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
        std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
        std::vector<pcl::ModelCoefficients> model_coefficients;
        std::vector<pcl::PointIndices> inlier_indices;  
        pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
        std::vector<pcl::PointIndices> label_indices;
        std::vector<pcl::PointIndices> boundary_indices;
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


        // Store result
        {
          boost::mutex::scoped_lock lock (vis_mutex);
          // Store the latest results for the visualizer
          vis_cloud_ = stage2_cloud_;
          vis_normals_ = stage2_normals_;
          vis_regions_ = stage3_regions_;
          vis_occluding_cloud_ = stage3_occluding_cloud_;
          // Update completed stage1 to stage2
          stage2_cloud_ = stage1_cloud_;
          stage2_normals_ = stage1_normals_;
          updated_data_ = true;
        }
      }
      
    }

    // Compute the normals
    template <typename PointT> void
    OrganizedFeatureExtraction<PointT>::computeNormals ()
    {
      ne.compute (*stage1_normals_);
    }

    // Compute planes
    template <typename PointT> void
    OrganizedFeatureExtraction<PointT>::computePlanes ()
    {
      if (stage2_cloud_->points.size () == 0)
        return;
      
      //std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
      //mps.segment (regions);
      mps.segment (stage3_regions_);
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
        
        CloudConstPtr cloud;
        CloudConstPtr occluding_cloud;
        NormalCloudConstPtr normals;
        std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;    

        if (vis_mutex.try_lock ())
        {
          vis_cloud_.swap (cloud);
          vis_occluding_cloud_.swap (occluding_cloud);
          vis_normals_.swap (normals);
          regions = vis_regions_;
          
          vis_mutex.unlock ();
        }
        
        if (cloud)
        {
          FPS_CALC ("visualization");
          //planar_region_callback_ (regions);
          if (updated_data_)
          {
            if (occluding_cloud->points.size () > 200)
            {
              printf ("CALLING OCCLUDING EDGE CALLBACK with %d points!\n", occluding_cloud->points.size ());
              std::cout << " OCCLUDING CLOUD STAMP: " << cloud->header.stamp << std::endl;
              occluding_edge_callback_ (occluding_cloud);
            }
            else
            {
              printf ("TOO FEW POINTS TO CALL OCCLUDING EDGE CALLBACK\n");
            }
            
            updated_data_ = false;
          }
          //if (!viewer_->updatePointCloud (prev_vis_cloud_, "cloud"))
          //  viewer_->addPointCloud (prev_vis_cloud_, "cloud");
          //if (!viewer_->updatePointCloud (cloud, "cloud"))
          //  viewer_->addPointCloud (cloud, "cloud");

          //viewer_->removePointCloud ("normals");
          //viewer_->addPointCloudNormals<PointT, pcl::Normal>(cloud, normals, 10, 0.05f, "normals");

          // Image Viewer
          //image_viewer_->addRGBImage<PointT>(cloud,"rgb_image");
          //displayPlanarRegions (regions, viewer_, image_viewer_);
        }
        
        //image_viewer_->spinOnce (100);

      }
      process_thread.join ();
    }

/*
    void
    displayPlanarRegions (std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > &regions, 
                          boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
                          boost::shared_ptr<pcl::visualization::ImageViewer> image_viewer)
    {
      char name[1024];
      unsigned char red [6] = {255,   0,   0, 255, 255,   0};
      unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
      unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};
      
      CloudPtr contour (new pcl::PointCloud<PointT>);
      
      for (size_t i = 0; i < regions.size (); i++)
      {
        Eigen::Vector3f centroid = regions[i].getCentroid ();
        Eigen::Vector4f model = regions[i].getCoefficients ();
        pcl::PointXYZ pt1 = pcl::PointXYZ (centroid[0], centroid[1], centroid[2]);
        pcl::PointXYZ pt2 = pcl::PointXYZ (centroid[0] + (0.5f * model[0]),
                                           centroid[1] + (0.5f * model[1]),
                                           centroid[2] + (0.5f * model[2]));
        sprintf (name, "normal_%d", unsigned (i));
        viewer->addArrow (pt2, pt1, 1.0, 0, 0, false, name);
        
        contour->points = regions[i].getContour ();
        sprintf (name, "plane_%02d", int (i));
        //pcl::visualization::PointCloudColorHandlerCustom <PointT> color (contour, red[i%6], grn[i%6], blu[i%6]);
        //if(!viewer->updatePointCloud(contour, color, name))
        //  viewer->addPointCloud (contour, color, name);
        //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
        image_viewer->addRGBImage<PointT>(contour, name, 0.3);
      }
    }
*/

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

    
}

//Instantiate
//template class omnimapper::OrganizedFeatureExtraction<pcl::PointXYZ>;
template class omnimapper::OrganizedFeatureExtraction<pcl::PointXYZRGBA>;


// int
// main (int argc, char** argv)
// {
//   pcl::OpenNIGrabber grabber ("#1");
  
//   XInitThreads ();

//   OrganizedFeatureExtraction<PointT> feature_test (grabber);
//   boost::this_thread::sleep (boost::posix_time::milliseconds (1000));
//   feature_test.spin ();
  
//   return 0;
// }
