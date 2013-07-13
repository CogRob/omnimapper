#include <omnimapper/object_plugin.h>
#include <omnimapper/plane.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

namespace omnimapper
{
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template <typename PointT>
  ObjectPlugin<PointT>::ObjectPlugin (omnimapper::OmniMapperBase* mapper)
    : mapper_ (mapper),
      observations_ (),
      empty_ ()
  {
    printf ("In constructor, checking size of observations_\n");
    printf ("Size: %d\n", observations_.size ());
  }
  
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template <typename PointT>
  ObjectPlugin<PointT>::~ObjectPlugin ()
  {
  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template <typename PointT> void
  ObjectPlugin<PointT>::clusterCloudCallback (std::vector<CloudPtr> clusters, omnimapper::Time t)
  {
    double max_cluster_dist_ = 3.0;
    double max_bbox_volume_ = 1.0;
    double max_bbox_dim_ =  1.0;
    double min_curvature_ = 0.01;
    double min_points_ = 1000;
    double min_clust_centroid_ptp_dist = 0.5;
    double ptp_pt_cull_thresh_ = 0.02;

    bool filter_points_near_planes_ = true;

    printf ("Object plugin got %d clusters\n", clusters.size ());
    // Get pose symbol for this timestamp
    gtsam::Symbol pose_symbol;
    mapper_->getPoseSymbolAtTime (t, pose_symbol);

    gtsam::Values solution;
    if (filter_points_near_planes_)
    {
      solution = mapper_->getSolution ();
    }
    gtsam::Values::ConstFiltered<gtsam::Plane<PointT> > plane_filtered = solution.filter<gtsam::Plane<PointT> >();

    // Save observations made from this pose
    //observations_.insert (std::pair<gtsam::Symbol, CloudPtrVector >(pose_symbol, clusters));
    CloudPtrVector filtered_observations;

    // Compute cluster statistics for filtering purposes
    Eigen::Matrix3f clust_cov;
    Eigen::Vector4f clust_centroid = Eigen::Vector4f::Zero ();
    Eigen::Vector4f min_pt;
    Eigen::Vector4f max_pt;
    EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
    EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
    
    for (int i = 0; i < clusters.size (); i++)
    {
      // Detect nearby planes
      if (filter_points_near_planes_)
      {
        // See what plane (if any) is closest
        Eigen::Vector4f clust_centroid;
        pcl::compute3DCentroid ((*(clusters[i])), clust_centroid);
        PointT clust_centroid_pt;
        clust_centroid_pt.x = clust_centroid[0];
        clust_centroid_pt.y = clust_centroid[1];
        clust_centroid_pt.z = clust_centroid[2];

        bool found_nearby_plane = false;
        //gtsam::Symbol closest_sym;
        std::vector<gtsam::Symbol> nearby_symbols;
        std::vector<Eigen::Vector4f> nearby_coeffs;
        BOOST_FOREACH (const typename gtsam::Values::ConstFiltered<gtsam::Plane<PointT> >::KeyValuePair& key_value, plane_filtered)
        {
          Cloud boundary_cloud = key_value.value.hull ();
          double ptp_dist = fabs (key_value.value.a () * clust_centroid[0] + 
                                  key_value.value.b () * clust_centroid[1] +
                                  key_value.value.c () * clust_centroid[2] + key_value.value.d ());
          bool pt_in_poly = pcl::isPointIn2DPolygon (clust_centroid_pt, boundary_cloud);
          if (pt_in_poly && (ptp_dist < min_clust_centroid_ptp_dist))
          {
            found_nearby_plane = true;
            //closest_sym = key_value.key;
            nearby_symbols.push_back (key_value.key);
            nearby_coeffs.push_back (Eigen::Vector4f (key_value.value.a (),
                                                      key_value.value.b (),
                                                      key_value.value.c (),
                                                      key_value.value.d ()));
          }
        }

        // If we found a nearby plane, remove points that are very close to it
        if (found_nearby_plane)
        {
          //gtsam::Plane<PointT> plane = solution.at<gtsam::Plane<PointT> > (closest_sym);
          CloudPtr filtered_clust (new Cloud());
          for (int j = 0; j < clusters[i]->points.size (); j++)
          {
            bool pt_ok = true;
            
            for (int k = 0; k < nearby_symbols.size (); k++)
            {
              double ptp_dist = fabs (nearby_coeffs[k][0] * clusters[i]->points[j].x +
                                      nearby_coeffs[k][1] * clusters[i]->points[j].y +
                                      nearby_coeffs[k][2] * clusters[i]->points[j].z +
                                      nearby_coeffs[k][3]);

              if (ptp_dist <= ptp_pt_cull_thresh_)
                pt_ok = false;
              
            // if (ptp_dist >= ptp_pt_cull_thresh_)
            // {
            //   filtered_clust->points.push_back (clusters[i]->points[j]);
            // }
            }

            if (pt_ok)
              filtered_clust->points.push_back (clusters[i]->points[j]);
          }
          printf ("cluster %d had %d points, filtered has %d points\n", i, clusters[i]->points.size (), 
                  filtered_clust->points.size ());
          clusters[i] = filtered_clust;
          printf ("cluster %d now has %d points\n", i, clusters[i]->points.size ());
        }
      }

      // If we still have enough points, filter based on entire cluster
      if (clusters[i]->points.size () >= min_points_)
      {
        // Get mean and covariance matrix
        pcl::computeMeanAndCovarianceMatrix ((*(clusters[i])), clust_cov, clust_centroid);
        
        double dist = sqrt(clust_centroid[0] * clust_centroid[0] + clust_centroid[1] * clust_centroid[1] + clust_centroid[2] * clust_centroid[2]);

        // Get bounding box
        pcl::getMinMax3D ((*(clusters[i])), min_pt, max_pt);
        Eigen::Vector4f size = max_pt - min_pt;
        double bbox_volume = size[0] * size[1] * size[2];
        
        // Get curvature
        pcl::eigen33 (clust_cov, eigen_value, eigen_vector);
        double eig_sum = clust_cov.coeff (0) + clust_cov (4) + clust_cov (8);
        double curvature = 0;
        if (eig_sum != 0)
          curvature = fabsf (eigen_value / eig_sum);
        
        printf ("Cluster %d:\n", i);
        printf ("  Dist: %lf\n", dist);
        printf ("  Volume: %lf\n", bbox_volume);
        printf ("  Dims: %lf, %lf, %lf\n", size[0], size[1], size[2]);
        printf ("  Curvature: %lf\n", curvature);
        
        // Clusters that are too far away suffer from too much depth discretization to be reliable
        bool dist_ok = dist < max_cluster_dist_;
        // Clusters with large bounding boxes are probably not objects of interest -- these tend to architectural features
        bool bbox_volume_ok = bbox_volume < max_bbox_volume_;
        // Clusters that have some dimension that is quite large tend to be uninteresting
        bool bbox_dims_ok = (size[0] < max_bbox_dim_) && (size[1] < max_bbox_dim_) && (size[2] < max_bbox_dim_);
        // Clusters with too low curvature are probably either planar objects that were improperly segmented, or
        // Also, if it is mostly planar, shape descriptors will not be very reliable
        bool curvature_ok = curvature > min_curvature_;
        
        printf ("centroid dist: %lf\n", dist);
        if (dist_ok && bbox_volume_ok && bbox_dims_ok && curvature_ok)
          filtered_observations.push_back (clusters[i]);
      }
      
    }
    

    observations_.insert (std::pair<gtsam::Symbol, CloudPtrVector>(pose_symbol, filtered_observations));

  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template <typename PointT> typename omnimapper::ObjectPlugin<PointT>::CloudPtrVector
  ObjectPlugin<PointT>::getObservations (gtsam::Symbol sym)
  {
    printf ("Starting getObservations!\n");
    printf ("Checking size...\n");
    if (observations_.size () == 0)
    {
      printf ("returning empty due to size!\n");
      return (empty_);
    }

    printf ("Checking count!\n");
    if (observations_.count (sym) > 0)
    {
      printf ("ObjectPlugin::getObservations: returning vec!\n");
      return (observations_.at (sym));
    }
    else
    {
      printf ("ObjectPlugin::getObservations: returning empty!\n");
      return (empty_);
    }
  }
  

  
}

// Instantiate
template class omnimapper::ObjectPlugin<pcl::PointXYZRGBA>;
