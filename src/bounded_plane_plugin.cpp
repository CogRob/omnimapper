#include <omnimapper/bounded_plane_plugin.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <omnimapper/geometry.h>

namespace omnimapper
{
  template <typename PointT>
  BoundedPlanePlugin<PointT>::BoundedPlanePlugin (omnimapper::OmniMapperBase* mapper)
    : mapper_ (mapper),
      max_plane_id_ (0)
  {
    printf ("BoundedPlanePlugin: Constructor.\n");
  }

  template <typename PointT> bool
  BoundedPlanePlugin<PointT>::polygonsOverlap (CloudPtr boundary1, CloudPtr boundary2)
  {
    // TODO: Make this use boost::geometry
    for (size_t i = 0; i < boundary1->points.size (); i++)
    {
      //if (pcl::isXYPointIn2DXYPolygon (boundary1.points[i], boundary2))
      if (pcl::isPointIn2DPolygon (boundary1->points[i], *boundary2))
        return true;
    }
    
    for (size_t i = 0; i < boundary2->points.size (); i++)
    {
      //if (pcl::isXYPointIn2DXYPolygon (boundary2.points[i], boundary1))
      if (pcl::isPointIn2DPolygon (boundary2->points[i], *boundary1))
        return true;
    }
    
    return false;
  }

  template <typename PointT> void
  BoundedPlanePlugin<PointT>::regionsToMeasurements (std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >& regions, omnimapper::Time t, std::vector<omnimapper::BoundedPlane3<PointT> >& plane_measurements)
  {
    // If we have a sensor_to_base_transform function, use it and apply the transform to incoming measurements
    Eigen::Affine3d sensor_to_base;
    bool use_transform = false;
    if (get_sensor_to_base_)
    {
      use_transform = true;
      sensor_to_base = (*get_sensor_to_base_)(t);
    }
    
    for (size_t i = 0; i < regions.size (); ++i)
    {
      Eigen::Vector4f model = regions[i].getCoefficients ();
      Eigen::Vector3f centroid = regions[i].getCentroid ();
      Eigen::Vector4f centroid4f (centroid[0], centroid[1], centroid[2], 0.0);
      //pcl::PointCloud<PointT> border_cloud;
      CloudPtr border_cloud (new Cloud ());
      std::vector<PointT, Eigen::aligned_allocator<PointT> > border = regions[i].getContour ();
      border_cloud->points = border;
      pcl::PointCloud<PointT> empty_inliers;

      // TODO : remove debug
      //PointVector poly(border);
      bool intersects = boost::geometry::intersects(border);
      if (intersects)
        printf("BoundedPlanePlugin: regions->measurements: GOT INVALID MEASUREMENT!\n");
      else
        printf("BoundedPlanePlugin: GOT VALID MEASUREMENT!\n");
      // Remove debug

      // Make a Plane      
      if (use_transform)
      {
        Eigen::Vector3d trans = sensor_to_base.inverse ().translation ();
        Eigen::Vector4f centroid4f_base = sensor_to_base.cast<float>() * centroid4f;
        Eigen::Vector3d model_norm (model[0], model[1], model[2]);
        Eigen::Vector3d model_norm_rot = sensor_to_base.linear () * model_norm;
        double d = model[0] * trans[0] + model[1] * trans[1] + model[2] * trans[2] + model[3];
        Eigen::Vector4f model_base (model_norm_rot[0], model_norm_rot[1], model_norm_rot[2], d);
        //model_base[3] = model_base.dot (centroid4f_base);

        Eigen::Vector4f vp = -centroid4f_base;
        float cos_theta = vp.dot (model_base);
        if (cos_theta < 0)
        {
          model_base *= -1;
          model_base[3] = 0;
          model_base[3] = -1 * model_base.dot (centroid4f_base);
        }

        printf ("Model: %lf %lf %lf %lf, Base Model: %lf %lf %lf %lf\n", model[0], model[1], model[2], model[3],
                model_base[0], model_base[1], model_base[2], model_base[3]);
        CloudPtr border_base (new Cloud ());
        pcl::transformPointCloud (*border_cloud, *border_base, sensor_to_base);
        //gtsam::Plane<PointT> plane (model_base[0], model_base[1], model_base[2], model_base[3], border_base, empty_inliers, centroid4f_base);
        omnimapper::BoundedPlane3<PointT> plane (model_base[0], model_base[1], model_base[2], model_base[3], border_base);
        plane_measurements.push_back (plane);
      }
      else 
      { 
        omnimapper::BoundedPlane3<PointT> plane (model[0], model[1], model[2], model[3], border_cloud);
        //gtsam::Plane<PointT> plane (model[0], model[1], model[2], model[3], border_cloud, empty_inliers, centroid4f);
      plane_measurements.push_back (plane);
      }
    }
  }

  template <typename PointT> void
  BoundedPlanePlugin<PointT>::planarRegionCallback (std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions, omnimapper::Time t)
  {
    printf ("BoundedPlanePlugin: Got %d regions.\n", regions.size ());
    
    // Convert the regions to omnimapper::BoundedPlane3
    std::vector<omnimapper::BoundedPlane3<PointT> > plane_measurements;
    regionsToMeasurements (regions, t, plane_measurements);
    printf ("BoundedPlanePlugin: Have %d measurements\n", plane_measurements.size ());

    // Get the planes from the mapper
    gtsam::Values current_solution = mapper_->getSolutionAndUncommitted ();//mapper_->getSolution ();
    gtsam::Values::Filtered<omnimapper::BoundedPlane3<PointT> > plane_filtered = current_solution.filter<omnimapper::BoundedPlane3<PointT> >();
    //gtsam::Values::Filtered<gtsam::OrientedPlane3> plane_filtered = current_solution.filter<gtsam::OrientedPlane3>();
    
    // Get the pose symbol for this time
    gtsam::Symbol pose_sym;
    mapper_->getPoseSymbolAtTime (t, pose_sym);
    boost::optional<gtsam::Pose3> new_pose = mapper_->predictPose (pose_sym);
    printf ("BoundedPlanePlugin: Processing planes for pose %s\n", boost::lexical_cast<std::string>(pose_sym.key ()).c_str ());
    
    // TODO: if above didn't work
    if (!new_pose)
    {
      printf ("BoundedPlanePlugin: No pose yet at this time!  Error!\n");
      return;
    }

    gtsam::Pose3 new_pose_inv = new_pose->inverse ();
    Eigen::Matrix4f new_pose_inv_tform = new_pose->matrix ().cast<float>();

    for (int i = 0; i < plane_measurements.size (); i++)
    {
      double lowest_error = std::numeric_limits<double>::infinity ();
      gtsam::Symbol best_symbol = gtsam::Symbol ('p', max_plane_id_);
      
      omnimapper::BoundedPlane3<PointT> meas_plane = plane_measurements[i];
      Eigen::Vector3d meas_norm = meas_plane.normal ().unitVector ();
      double meas_d = meas_plane.d ();
      CloudPtr meas_boundary = meas_plane.boundary ();
      CloudPtr meas_boundary_map(new Cloud());
      Eigen::Affine3f pose2map = pose3ToTransform(*new_pose);
      pcl::transformPointCloud(*meas_boundary, *meas_boundary_map, pose2map);

      // TODO : remove debug
      //PointVector poly(border);
      bool valid = boost::geometry::intersects(meas_boundary_map->points);
      if (!valid)
        printf("BoundedPlanePlugin: map_meas: GOT INVALID MEASUREMENT!\n");
      // Remove debug
      
      if (meas_d < 0.1)
        continue;

//      BOOST_FOREACH (const typename gtsam::Values::Filtered<gtsam::OrientedPlane3>::KeyValuePair& key_value, plane_filtered)
      BOOST_FOREACH (const typename gtsam::Values::Filtered<omnimapper::BoundedPlane3<PointT> >::KeyValuePair& key_value, plane_filtered)
      {
        gtsam::Symbol key_symbol (key_value.key);
        //omnimapper::BoundedPlane3<PointT> plane = key_value.value;
        omnimapper::BoundedPlane3<PointT> plane = key_value.value;
        
        //gtsam::OrientedPlane3 predicted_plane = gtsam::OrientedPlane3::Transform (plane, (*new_pose), boost::none, boost::none);
        Eigen::Vector4d pred_coeffs = omnimapper::BoundedPlane3<PointT>::TransformCoefficients (plane, (*new_pose));//predicted_plane.planeCoefficients ();
        Eigen::Vector3d pred_norm (pred_coeffs[0], pred_coeffs[1], pred_coeffs[2]);
        double pred_d = pred_coeffs[3];
        
        double angular_error = acos (meas_norm.dot (pred_norm));
        double range_error = fabs (meas_d - pred_d);
        printf ("BoundedPlanePlugin: ang error: %lf range_error: %lf\n", angular_error, range_error);
        
        if ((angular_error < angular_threshold_) && (range_error < range_threshold_))
        {
          double error = angular_error + range_error;
          // polygon overlap
          if (polygonsOverlap(plane.boundary(), meas_boundary_map))

          if ((error < lowest_error))
          {
            lowest_error = error;
            best_symbol = key_symbol;
          }
        }
      }
      
      // Add factors
      if (lowest_error == std::numeric_limits<double>::infinity ())
      {
        //omnimapper::BoundedPlane3<PointT> map_p3 = gtsam::OrientedPlane3::Transform (meas_plane, *new_pose);
        //Eigen::Vector4d map_p3_coeffs = omnimapper::BoundedPlane3<PointT>::TransformCoefficients (meas_plane, *new_pose);//map_p3.planeCoefficients ();
        Eigen::Vector4d map_p3_coeffs = omnimapper::BoundedPlane3<PointT>::TransformCoefficients (meas_plane, new_pose_inv);
        //Eigen::Affine3f pose2map = pose3ToTransform(*new_pose);
        //CloudPtr map_boundary (new Cloud ());
        //pcl::transformPointCloud (*meas_boundary, *map_boundary, pose2map);
        //omnimapper::BoundedPlane3<PointT> map_plane (map_p3_coeffs[0], map_p3_coeffs[1], map_p3_coeffs[2], map_p3_coeffs[3], map_boundary);
        omnimapper::BoundedPlane3<PointT> map_plane (map_p3_coeffs[0], map_p3_coeffs[1], map_p3_coeffs[2], map_p3_coeffs[3], meas_boundary_map);

        //gtsam::Plane<PointT> new_plane (*new_pose, meas_plane, false);//(new_pose_inv, meas_plane, false);
        printf ("BoundedPlanePlugin: Creating new plane %s: %lf %lf %lf %lf\n", boost::lexical_cast<std::string>(best_symbol.key ()).c_str (), map_p3_coeffs[0], map_p3_coeffs[1], map_p3_coeffs[2], map_p3_coeffs[3]);
        mapper_->addNewValue (best_symbol, map_plane);
        ++max_plane_id_;
      }
      else 
      {
        // lock plane & update
        printf("BoundedPlanePlugin: Extending boundary...\n");
        mapper_->updateBoundedPlane(best_symbol, *new_pose, meas_plane);
        //omnimapper::BoundedPlane3<PointT> map_plane = current_solution.at<omnimapper::BoundedPlane3<PointT> > (best_symbol);
        //map_plane.extendBoundary((*new_pose), meas_plane);
        printf("BoundedPlanePlugin: Boundary Extended...\n");
      } 
        
      gtsam::SharedDiagonal measurement_noise;
      measurement_noise = gtsam::noiseModel::Diagonal::Sigmas ((gtsam::Vector (3) << angular_noise_, angular_noise_, range_noise_));
      
      gtsam::Vector measurement_vector = meas_plane.planeCoefficients ();
      omnimapper::OmniMapperBase::NonlinearFactorPtr plane_factor (new omnimapper::BoundedPlaneFactor<PointT> (measurement_vector, meas_boundary, measurement_noise, pose_sym, best_symbol));
      mapper_->addFactor (plane_factor);
      printf ("BoundedPlanePlugin: Added factor!\n");
    }// plane measurements

  }
  
}

template class omnimapper::BoundedPlanePlugin<pcl::PointXYZRGBA>;
