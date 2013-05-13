#include <omnimapper/plane_plugin.h>

namespace omnimapper
{
  template <typename PointT>
  PlaneMeasurementPlugin<PointT>::PlaneMeasurementPlugin (omnimapper::OmniMapperBase* mapper) 
    : mapper_ (mapper),
      max_plane_id_ (0),
      angular_threshold_ (0.017453*15.0),//7.0
      range_threshold_ (0.2),
      overwrite_timestamps_ (true)
  {
    
  }

  template <typename PointT> void
  PlaneMeasurementPlugin<PointT>::regionsToMeasurements (std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >& regions, std::vector<gtsam::Plane<PointT> >& plane_measurements)
  {
    for (size_t i = 0; i < regions.size (); ++i)
    {
      Eigen::Vector4f model = regions[i].getCoefficients ();
      Eigen::Vector3f centroid = regions[i].getCentroid ();
      Eigen::Vector4f centroid4f (centroid[0], centroid[1], centroid[2], 0.0);
      pcl::PointCloud<PointT> border_cloud;
      std::vector<PointT, Eigen::aligned_allocator<PointT> > border = regions[i].getContour ();
      border_cloud.points = border;
      pcl::PointCloud<PointT> empty_inliers;
      std_msgs::Header empty_header;
      
      // Make a Plane
      gtsam::Plane<PointT> plane (model[0], model[1], model[2], model[3], border_cloud, empty_inliers, centroid4f, empty_header);
      
      plane_measurements.push_back (plane);
    }
  }
  
  template <typename PointT> void
  PlaneMeasurementPlugin<PointT>::planarRegionCallback (std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >& regions, omnimapper::Time& t)
  {
    printf ("PlaneMeasurementPlugin: Got %d planes.\n", regions.size ());
    
    if (overwrite_timestamps_)
      t = boost::posix_time::ptime ( boost::posix_time::microsec_clock::local_time() );

    // Convert the regions to gtsam::Planes
    std::vector<gtsam::Plane<PointT> > plane_measurements;
    regionsToMeasurements (regions, plane_measurements);
    
    // Get the planes from the mapper
    gtsam::Values current_solution = mapper_->getSolutionAndUncommitted ();//mapper_->getSolution ();
    gtsam::Values::ConstFiltered<gtsam::Plane<PointT> > plane_filtered = current_solution.filter<gtsam::Plane<PointT> >();

    // Get the pose symbol for this time
    gtsam::Symbol pose_sym;
    mapper_->getPoseSymbolAtTime (t, pose_sym);
    boost::optional<gtsam::Pose3> new_pose = mapper_->predictPose (pose_sym);
    // TODO: if above didn't work
    if (!new_pose)
    {
      printf ("No pose yet at this time!  Error!\n");
      return;
    }

    gtsam::Pose3 new_pose_inv = new_pose->inverse ();

    // Data Association
    for (int i = 0; i < plane_measurements.size (); i++)
    {
      double lowest_error = std::numeric_limits<double>::infinity ();
      gtsam::Symbol best_symbol = gtsam::Symbol ('p', max_plane_id_);//plane_filtered.size ());

      gtsam::Plane<PointT> meas_plane = plane_measurements[i];
      Eigen::Vector3f meas_norm (meas_plane.a (), meas_plane.b (), meas_plane.c ());
      printf ("measurement: %lf %lf %lf %lf\n", meas_plane.a (), meas_plane.b (), meas_plane.c (), meas_plane.d ());

      printf ("plane_filtered size: %d\n", plane_filtered.size ());
      BOOST_FOREACH (const typename gtsam::Values::ConstFiltered<gtsam::Plane<PointT> >::KeyValuePair& key_value, plane_filtered)
      {
        gtsam::Symbol key_symbol (key_value.key);
        gtsam::Plane<PointT> plane = key_value.value;
      
        gtsam::Vector predicted_measurement = plane.GetXo (*new_pose);//(new_pose_inv);//(*new_pose);
        printf ("predicted_measurement: %lf %lf %lf %lf\n", predicted_measurement[0], predicted_measurement[1], predicted_measurement[2], predicted_measurement[3]);
        Eigen::Vector3f pred_norm (predicted_measurement[0], predicted_measurement[1], predicted_measurement[2]);
        //Eigen::Vector3f meas_norm (plane.a (), plane.b (), plane.c ());
      
        // Compute error
        double angular_error = acos (meas_norm.dot (pred_norm));
        double range_error = fabs (meas_plane.d () -  predicted_measurement[3]);
        printf ("ang error: %lf range_error: %lf\n", angular_error, range_error);
        
        // If the planar equations are similar, we need to additionally check for polygon overlap
        //if ((angular_error > angular_threshold_) && (range_error < range_threshold_))
        if ((angular_error < angular_threshold_) && (range_error < range_threshold_))
        {
          //double error = (1.0 - angular_error) + range_error;
          double error = angular_error + range_error;
        
          // Get the 
          //pcl::PointCloud<PointT> predicted_boundary;
          // TODO: Polygon Overlap Check!
          if (error < lowest_error)
          {
            printf ("new potential match to plane %d\n", key_symbol.index ());
            lowest_error = error;
            best_symbol = key_symbol;
          }
        }

      }

      // Add factors
      // If we didn't find a match, this is a new landmark
      if (lowest_error == std::numeric_limits<double>::infinity ())
      {
        gtsam::Plane<PointT> new_plane (*new_pose, meas_plane, false);//(new_pose_inv, meas_plane, false);
        printf ("Creating new plane!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        mapper_->addNewValue (best_symbol, new_plane);
        ++max_plane_id_;
      }
      
      // Add the measurement factor
      // TODO: non-bogus measurement noise
      gtsam::SharedDiagonal measurement_noise;
//      gtsam::SharedDiagonal noise = gtsam::noiseModel::Diagonal::Sigmas (gtsam::Vector_ (6, rot_noise, rot_noise, rot_noise, trans_noise, trans_noise, trans_noise));
      //measurement_noise = gtsam::noiseModel::Diagonal::Sigmas (gtsam::Vector_ (4, 0.01, 0.01, 0.01, 0.03));
      measurement_noise = gtsam::noiseModel::Diagonal::Sigmas (gtsam::Vector_ (4, 0.1, 0.1, 0.1, 0.2));
      
      gtsam::Vector measurement_vector = meas_plane.GetXf ();
      omnimapper::OmniMapperBase::NonlinearFactorPtr plane_factor(new gtsam::PlaneFactor<PointT> (measurement_vector, measurement_noise, pose_sym, best_symbol));
      mapper_->addFactor (plane_factor);
      printf ("Adding factor!\n");
      
    }
    

  }
}


// TODO: Instantiation macros.
template class omnimapper::PlaneMeasurementPlugin<pcl::PointXYZ>;
template class omnimapper::PlaneMeasurementPlugin<pcl::PointXYZRGBA>;
