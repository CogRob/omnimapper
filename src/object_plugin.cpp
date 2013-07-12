#include <omnimapper/object_plugin.h>

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

    printf ("Object plugin got %d clusters\n", clusters.size ());
    // Get pose symbol for this timestamp
    gtsam::Symbol pose_symbol;
    mapper_->getPoseSymbolAtTime (t, pose_symbol);

    // Save observations made from this pose
    //observations_.insert (std::pair<gtsam::Symbol, CloudPtrVector >(pose_symbol, clusters));
    CloudPtrVector filtered_observations;

    // Compute cluster statistics for filtering purposes
    Eigen::Matrix3f clust_cov;
    Eigen::Vector4f clust_centroid = Eigen::Vector4f::Zero ();
    Eigen::Vector4f min_pt;
    Eigen::Vector4f max_pt;
    
    for (int i = 0; i < clusters.size (); i++)
    {
      pcl::computeMeanAndCovarianceMatrix ((*(clusters[i])), clust_cov, clust_centroid);
      
      double dist = sqrt(clust_centroid[0] * clust_centroid[0] + clust_centroid[1] * clust_centroid[1] + clust_centroid[2] * clust_centroid[2]);

      pcl::getMinMax3D ((*(clusters[i])), min_pt, max_pt);
      
      Eigen::Vector4f size = max_pt - min_pt;

      double bbox_volume = size[0] * size[1] * size[2];
      
      bool dist_ok = dist < max_cluster_dist_;
      bool bbox_volume_ok = bbox_volume < max_bbox_volume_;
      bool bbox_dims_ok = (size[0] < max_bbox_dim_) && (size[1] < max_bbox_dim_) && (size[2] < max_bbox_dim_);

      printf ("centroid dist: %lf\n", dist);
      if (dist_ok && bbox_volume_ok && bbox_dims_ok)
        filtered_observations.push_back (clusters[i]);
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
