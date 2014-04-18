#include <omnimapper/bounded_plane_plugin.h>

namespace omnimapper
{
  template <typename PointT>
  BoundedPlanePlugin<PointT>::BoundedPlanePlugin (omnimapper::OmniMapperBase* mapper)
    : mapper_ (mapper),
      max_plane_id_ (0)
  {
    printf ("BoundedPlanePlugin: Constructor.\n");
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
    
    // Convert the regions to gtsam::Planes
    //std::vector<omnimapper::BoundedPlane3<PointT> > plane_measurements;
    //regionsToMeasurements (regions, t, plane_measurements);


    // Get the planes from the mapper
    gtsam::Values current_solution = mapper_->getSolutionAndUncommitted ();//mapper_->getSolution ();
    gtsam::Values::Filtered<omnimapper::BoundedPlane3<PointT> > plane_filtered = current_solution.filter<omnimapper::BoundedPlane3<PointT> >();
    
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

    
  }
  
}

template class omnimapper::BoundedPlanePlugin<pcl::PointXYZRGBA>;
