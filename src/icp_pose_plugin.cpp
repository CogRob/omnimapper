#include <omnimapper/icp_pose_plugin.h>
#include <omnimapper/omnimapper_base.h>
#include <omnimapper/time_utils.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>

namespace omnimapper 
{
  template <typename PointT>
  ICPPoseMeasurementPlugin<PointT>::ICPPoseMeasurementPlugin (omnimapper::OmniMapperBase* mapper, pcl::Grabber& grabber) :
    mapper_ (mapper),
    initialized_ (false),
    grabber_ (grabber),
    have_new_cloud_ (false),
    first_ (true),
    downsample_ (true),
    leaf_size_ (0.05f),
    debug_ (true),
    overwrite_timestamps_ (true),
    icp_max_correspondence_distance_ (3.5),
    previous_sym_ (gtsam::Symbol ('x', 0)),
    use_gicp_ (true),
    paused_ (false)
  {
    have_new_cloud_ = false;
    boost::function<void (const CloudConstPtr&)> f = boost::bind (&ICPPoseMeasurementPlugin<PointT>::cloudCallback, this, _1);
    boost::signals2::connection c = grabber_.registerCallback(f);
    grabber_.start ();
    first_ = true;
  }

  template <typename PointT> ICPPoseMeasurementPlugin<PointT>::~ICPPoseMeasurementPlugin ()
  {
  }

  template <typename PointT> void 
  ICPPoseMeasurementPlugin<PointT>::cloudCallback (const CloudConstPtr& cloud)
  {
    printf ("cloud callback\n");
    // Store this as the previous cloud
    boost::mutex::scoped_lock (current_cloud_mutex_);
    current_cloud_ = cloud;
    have_new_cloud_ = true;
    printf ("stored new cloud!\n");
  }

  /*
  template <typename PointT> bool
  ICPPoseMeasurementPlugin<PointT>::addInitialPose ()
  {
    // Do nothing if there isn't a new cloud
    if (!have_new_cloud_)
    {
      printf ("no new cloud!\n");
      return false;
    }

    gtsam::Symbol current_symbol = mapper_->currentPoseSymbol ();
    printf ("current symbol: %d\n", current_symbol.index ());
    {
      boost::mutex::scoped_lock (current_cloud_mutex_);
      clouds_.insert (std::pair<gtsam::Symbol, CloudConstPtr> (current_symbol, current_cloud_));
      
      // If this is the first cloud, just add it and we're done -- no motion has occured
      // Note that initialization of the first pose is done by the OmniMapperBase
      if (clouds_.size () == 1)
      {
        initialized_ = true;
        have_new_cloud_ = false;
        return (true);
      }
    }
  }
  */

  template <typename PointT> void
  ICPPoseMeasurementPlugin<PointT>::spin ()
  {
    //while (grabber_.isRunning ())
    while (true)
    {
      spinOnce ();
      boost::this_thread::sleep (boost::posix_time::milliseconds (10));
    }
  }

  template <typename PointT> bool
  ICPPoseMeasurementPlugin<PointT>::spinOnce ()
  {
    // Do nothing if there isn't a new cloud
    if (!have_new_cloud_)
    {
      printf ("ICPPoseMeasurementPlugin: no new cloud!\n");
      return false;
    }
    
    // Get a shared_ptr to the latest cloud
    CloudConstPtr current_cloud;// (new Cloud ());
    {
      boost::mutex::scoped_lock (current_cloud_mutex_);
      
      // Strict input checking
      if (false)
      {
        if (current_cloud_->points.size () < 200)
        { 
          printf ("ICPPoseMeasurementPlugin: Not enough points!\n");
          return (false);
        }

        for (int i = 0; i < current_cloud_->points.size (); i++)
        {
          if (!pcl::isFinite (current_cloud_->points[i]))
          {
            printf ("NAN FOUND IN INPUT!");
            return (false);
          }
        }
        

      }
      

      current_cloud = current_cloud_;
    }
    printf ("current cloud points: %d\n", current_cloud->points.size ());
    
    
    // Downsample, if needed
    CloudPtr current_cloud_filtered (new Cloud ());
    if (downsample_)
    {
      pcl::VoxelGrid<PointT> grid;
      grid.setLeafSize (leaf_size_, leaf_size_, leaf_size_);
      grid.setInputCloud (current_cloud);
      grid.filter (*current_cloud_filtered);
    }
    else
    {
      *current_cloud_filtered = *current_cloud;
    }

    // Get the previous pose and cloud
    //gtsam::Symbol current_sym = mapper_->currentPoseSymbol ();
    gtsam::Symbol current_sym;
    boost::posix_time::ptime current_time;
    if (overwrite_timestamps_)
    {
      current_time = boost::posix_time::ptime ( boost::posix_time::microsec_clock::local_time() );
    }
    else
    {
#ifdef USE_ROS
      current_time = current_cloud->header.stamp.toBoost ();//current_cloud_->header.stamp.toBoost ();
#else
      current_time = boost::posix_time::ptime( omnimapper::stamp2ptime (current_cloud->header.stamp));//(current_cloud_->header.stamp) );
#endif //USE_ROS
    }
    mapper_->getPoseSymbolAtTime (current_time, current_sym);
    //std::cout << "stamp time: " << current_cloud_->header.stamp << " converted time: " << current_time << std::endl;
    printf ("current symbol: %d\n", current_sym.index ());
    //{
    //  boost::mutex::scoped_lock (current_cloud_mutex_);
    clouds_.insert (std::pair<gtsam::Symbol, CloudConstPtr> (current_sym, current_cloud_filtered));//current_cloud_));
    //}
  
    // We're done if that was the first cloud
    printf ("first: %d\n", first_);
    if (first_)
    {
      printf ("done with first, returning\n");
      have_new_cloud_ = false;
      previous_sym_ = current_sym;
      first_ = false;
      return (false);
    }
    
    // Register the clouds
    // TODO: we should look up the initial guess from the mapper
    CloudPtr aligned_cloud (new Cloud ());
    Eigen::Matrix4f cloud_tform = Eigen::Matrix4f::Identity ();
    double icp_score = 0.0;
    printf ("At: %d size: %d\n", previous_sym_.index (), clouds_.size ());
    // Debug
    if (current_sym == previous_sym_)
      assert (false);
    
    bool icp_converged = registerClouds (clouds_.at (current_sym), clouds_.at (previous_sym_), aligned_cloud, cloud_tform, icp_score);
    printf ("Done with registration\n");

    if (icp_converged)// && icp_score >= 0.01)
    {
      // Add a between factor from the previous pose to this pose
      gtsam::Pose3 relative_pose (gtsam::Rot3 (cloud_tform.block (0, 0, 3, 3).cast<double>()), 
                                  gtsam::Point3 (cloud_tform (0,3), cloud_tform (1,3), cloud_tform (2,3)));
      // The transform is the inverse of what we need
      //cloud_tform = cloud_tform.inverse ().eval ();
      relative_pose = relative_pose.inverse ();

      // TODO: make these params
      double trans_noise = 1.0 * icp_score;
      double rot_noise = 1.0 * icp_score;
      gtsam::SharedDiagonal noise = gtsam::noiseModel::Diagonal::Sigmas (gtsam::Vector_ (6, rot_noise, rot_noise, rot_noise, trans_noise, trans_noise, trans_noise));
      //gtsam::BetweenFactor<gtsam::Pose3> between (previous_sym, current_sym, relative_pose, noise);
      //mapper_->new_factors.add (between);
      omnimapper::OmniMapperBase::NonlinearFactorPtr between (new gtsam::BetweenFactor<gtsam::Pose3> (previous_sym_, current_sym, relative_pose, noise));
      mapper_->addFactor (between);
      
      // If init, initialize the pose at our result
      // if (init)
      // {
      //   // Look up the most recent location for new_pose
      //   gtsam::Pose3 new_pose = mapper_->getPose (previous_sym).compose (relative_pose);
      //   mapper_->addNewValue (current_sym, new_pose);
      // }
    } 
    else 
    {
      // Else put identity
      cloud_tform = Eigen::Matrix4f::Identity ();
      gtsam::Pose3 relative_pose (gtsam::Rot3 (cloud_tform.block (0, 0, 3, 3).cast<double>()), 
                                  gtsam::Point3 (cloud_tform (0,3), cloud_tform (1,3), cloud_tform (2,3)));
      double trans_noise = 1.0;
      double rot_noise = 1.0;
      gtsam::SharedDiagonal noise = gtsam::noiseModel::Diagonal::Sigmas (gtsam::Vector_ (6, rot_noise, rot_noise, rot_noise, trans_noise, trans_noise, trans_noise));
      omnimapper::OmniMapperBase::NonlinearFactorPtr between (new gtsam::BetweenFactor<gtsam::Pose3> (previous_sym_, current_sym, relative_pose, noise));
      mapper_->addFactor (between);
    }
    
    
      
    // Note that we're done
    {
      boost::mutex::scoped_lock (current_cloud_mutex_);
      have_new_cloud_ = false;
      previous_sym_ = current_sym;
    }

    if (debug_)
      printf ("ICPPoseMeasurementPlugin: Added a pose!\n");
    return (true);
  }

  template <typename PointT> bool
  ICPPoseMeasurementPlugin<PointT>::registerClouds (CloudConstPtr& cloud1, CloudConstPtr& cloud2, CloudPtr& aligned_cloud2, Eigen::Matrix4f& tform, double& score)
  {
    printf ("Starting icp... Cloud1: %d Cloud2: %d\n", cloud1->points.size (), cloud2->points.size ());
    std::cout << "Cloud1 stamp: " << cloud1->header.stamp << " Cloud2 stamp: " << cloud2->header.stamp << std::endl;
    //pcl::IterativeClosestPointNonLinear<PointT, PointT> icp;
    //pcl::IterativeClosestPoint<PointT, PointT> icp;
    if (use_gicp_)
    {
      pcl::GeneralizedIterativeClosestPoint<PointT, PointT> icp;
      icp.setMaximumIterations (100);//20
      icp.setTransformationEpsilon (1e-6);
      icp.setMaxCorrespondenceDistance (icp_max_correspondence_distance_);//1.5
      icp.setInputCloud (cloud2);
      icp.setInputTarget (cloud1);
      icp.align (*aligned_cloud2, tform);
      printf ("ICP completed...\n");
      tform = icp.getFinalTransformation ();
      score = icp.getFitnessScore ();
    }
    else
    {
      pcl::IterativeClosestPoint<PointT, PointT> icp;
      icp.setMaximumIterations (100);//20
      icp.setTransformationEpsilon (1e-6);
      icp.setMaxCorrespondenceDistance (icp_max_correspondence_distance_);//1.5
      icp.setInputCloud (cloud2);
      icp.setInputTarget (cloud1);
      icp.align (*aligned_cloud2, tform);
      printf ("ICP completed...\n");
      tform = icp.getFinalTransformation ();
      score = icp.getFitnessScore ();
    }
    

    std::cout << "has converged score: " << score << std::endl;
    printf ("tform:\n%lf %lf %lf %lf\n",tform (0,0), tform (0,1), tform (0,2), tform (0,3));
    //score = icp.getFitnessScore ();
  
    // if (!icp.hasConverged ())
    // {
    //   printf ("ICP failed to converge, skipping this cloud!\n");
    //   return false;
    // }
  
    return true;
  }

  template <typename PointT> bool
  ICPPoseMeasurementPlugin<PointT>::ready ()
  {
    boost::mutex::scoped_lock (current_cloud_mutex_);
    return (have_new_cloud_);
  }

  template <typename PointT> void
  ICPPoseMeasurementPlugin<PointT>::pause (bool pause)
  { 
    paused_ = pause;
    if (paused_)
    { 
      grabber_.stop ();
    } 
    else 
    { 
      grabber_.start ();
    }
  }

  template <typename PointT> typename omnimapper::ICPPoseMeasurementPlugin<PointT>::CloudConstPtr
  ICPPoseMeasurementPlugin<PointT>::getCloudPtr (gtsam::Symbol sym)
  {
    printf ("ICPPlugin: In getCloudPtr!\n");
    if (clouds_.count (sym) > 0)
      return (clouds_.at (sym));
    else
    {
      printf ("ERROR: REQUESTED SYMBOL WITH NO POINTS!\n");
      CloudConstPtr empty (new Cloud ());
      return (empty);
    }
  }
  
}

// TODO: Instantiation macros.
template class omnimapper::ICPPoseMeasurementPlugin<pcl::PointXYZ>;
//template class ICPPoseMeasurementPlugin<pcl::PointXYZRGB>;
template class omnimapper::ICPPoseMeasurementPlugin<pcl::PointXYZRGBA>;
