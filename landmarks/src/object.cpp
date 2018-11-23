#include <omnimapper/object.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

namespace omnimapper {

template<typename PointT>
Object<PointT>::Object(gtsam::Symbol object_symbol) :
		clusters_(), landmark(false), unoptimal_cloud_(new Cloud()), optimal_cloud_(new Cloud()), debug_(true), verbose_(true){
  sym = object_symbol;
}


template<typename PointT>
Object<PointT>::Object(const Object<PointT>& object):
      clusters_ (object.clusters_), landmark (object.landmark), optimal_cloud_ (
          object.optimal_cloud_), unoptimal_cloud_(object.unoptimal_cloud_), factor_flag(object.factor_flag),
          indices_(object.indices_),sym(object.sym), debug_(true)
{
}

template<typename PointT>
void Object<PointT>::addObservation(gtsam::Symbol pose_sym, CloudPtr cluster, gtsam::Pose3 cloud_pose) {

  boost::lock_guard<boost::mutex> lock (object_mutex_);

  if(verbose_)
   printf("[Object] adding observation for object %c%d from frame pose %c%d\n",sym.chr(), sym.index(), pose_sym.chr(), pose_sym.index());

  clusters_.insert (
        std::make_pair (pose_sym, std::make_pair (cloud_pose, cluster)));

  PoseCloud pose_cloud = clusters_.at (pose_sym);
  CloudPtr cloud = pose_cloud.second;
  Cloud map_cloud;

  if(verbose_)
    printf("[Object] object segment added with size %d\n", cluster->points.size());

  gtsam::Pose3 new_pose = cloud_pose;
  Eigen::Matrix4f map_transform = new_pose.matrix ().cast<float> ();
  pcl::transformPointCloud (*cluster, map_cloud, map_transform);
  *unoptimal_cloud_ = *unoptimal_cloud_ + map_cloud;


  if(verbose_)
      printf("[Object] Size of cluster %d unoptimal cloud updated %d\n", cluster->points.size(), unoptimal_cloud_->points.size());

  pcl::compute3DCentroid(*unoptimal_cloud_, centroid_);

  Eigen::Vector4f min_pt, max_pt;
  pcl::getMinMax3D(*unoptimal_cloud_, min_pt, max_pt);
  bounding_box_ = std::make_pair(min_pt, max_pt);

  factor_flag.insert (
      std::pair<gtsam::Symbol, int> (pose_sym, -1));

  if(verbose_)
      printf("[Object] done addObservation\n");


}


template<typename PointT>
typename Object<PointT>::Cloud Object<PointT>::optimizedCloud()
{
  boost::lock_guard<boost::mutex> lock (object_mutex_);
  Cloud cloud = *optimal_cloud_;
  return cloud;
}

template<typename PointT>
typename Object<PointT>::Cloud Object<PointT>::unOptimizedCloud()
{
  boost::lock_guard<boost::mutex> lock (object_mutex_);
  if(verbose_)
    printf("[Object] Inside unOptimizedCloud\n");
  Cloud cloud = *unoptimal_cloud_;
  if(verbose_)
      printf("[Object] Size of cloud %d\n", cloud.points.size());
  return cloud;
}

  template<typename PointT>
  void Object<PointT>::updateCentroid (CloudPtr cloud)
  {
    boost::lock_guard<boost::mutex> lock (object_mutex_);
    pcl::compute3DCentroid(*cloud, centroid_);
    return;
  }

  template<typename PointT>
  void Object<PointT>::updateBoundingBox (CloudPtr cloud)
  {
  boost::lock_guard<boost::mutex> lock (object_mutex_);
  Eigen::Vector4f min_pt, max_pt;
  pcl::getMinMax3D(*cloud, min_pt, max_pt);
  bounding_box_ = std::make_pair(min_pt, max_pt);
  return;
  }


  template<typename PointT>
  void Object<PointT>::saveAsPCD (std::string location)
  {
  boost::lock_guard<boost::mutex> lock (object_mutex_);
  std::string fname = location + "/" +boost::lexical_cast<std::string>(sym.chr()) + boost::lexical_cast<std::string> (sym.index()) + ".pcd";
  pcl::io::savePCDFileASCII (fname, *unoptimal_cloud_);
  return;

  }


  template<typename PointT>
  typename Object<PointT>::CloudPtr
  Object<PointT>::updateOptimizedCloud ()
  {
    boost::lock_guard<boost::mutex> lock (object_mutex_);
      optimal_cloud_->clear();
      CloudPtr transformed_cloud_opt(new Cloud());
      typename std::map<gtsam::Symbol, PoseCloud >::iterator it_cluster;
      for (it_cluster = clusters_.begin (); it_cluster != clusters_.end ();
          it_cluster++)
      {
      gtsam::Symbol pose_sym = it_cluster->first;  // robot pose symbol
      PoseCloud pose_cloud = it_cluster->second;  // cloud pointer
      gtsam::Pose3 cloud_pose = pose_cloud.first;
      CloudPtr map_cloud (new Cloud ());
      Eigen::Matrix4f map_transform = cloud_pose.matrix ().cast<float> ();
      pcl::transformPointCloud (*(pose_cloud.second), *map_cloud, map_transform);  // transform cloud from base_frame to world_frame
      *transformed_cloud_opt = *transformed_cloud_opt + *map_cloud;
      }
      optimal_cloud_ = transformed_cloud_opt;
      unoptimal_cloud_ = optimal_cloud_;

      return optimal_cloud_;
  }


  template<typename PointT>
  typename Object<PointT>::CloudPtr
  Object<PointT>::updateUnoptimizedCloud (gtsam::Symbol sym_latest, gtsam::Pose3 cloud_pose)
  {
    boost::lock_guard<boost::mutex> lock (object_mutex_);
    gtsam::Symbol sym = sym_latest;
    PoseCloud pose_cloud = clusters_.at (sym);
    CloudPtr map_cloud(new Cloud());

    gtsam::Pose3 new_pose = cloud_pose;
    Eigen::Matrix4f map_transform = new_pose.matrix ().cast<float> ();
    pcl::transformPointCloud (*(pose_cloud.second), *map_cloud, map_transform);
    *unoptimal_cloud_ = *unoptimal_cloud_ + *map_cloud;
    return unoptimal_cloud_;

  }



//   template<typename PointT>
//   void Object<PointT>::computeTSDF (Eigen::Vector4f obj_centroid)
//   {
//     if(verbose_)
//     printf ("[ObjectPlugin] starting generateTSDF\n");

//     gtsam::Symbol object_symbol = object.sym;
//     int id = object_symbol.index();

// // Make a TSDF

//     tsdf->setGridSize (10.0, 10.0, 10.0);
//     tsdf->setResolution (2048, 2048, 2048);

//     Eigen::Affine3d object_center;
//     object_center.translation () = Eigen::Vector3d (obj_centroid[0],
//         obj_centroid[1], obj_centroid[2]);
//     Eigen::Affine3d tsdf_center = Eigen::Affine3d::Identity ();  // Optionally offset the center
//     tsdf->setGlobalTransform (object_center);
// //tsdf->setDepthTruncationLimits ();
// //tsdf->setDepthTruncationLimits (0.3, 10.0);
// //tsdf->setWeightTruncationLimit (100.0);
//     tsdf->reset ();  // Initialize it to be empty

//     std::map<gtsam::Symbol, CloudPtr> cluster = clusters_;
//     std::map<gtsam::Symbol, pcl::PointIndices> indices = indices_;

//     typename std::map<gtsam::Symbol, CloudPtr>::iterator it;

//     Cloud transformed_cloud_opt_;

//     for (it = cluster.begin (); it != cluster.end (); it++)
//     {

//       gtsam::Symbol sym = it->first;
//       CloudPtr cloud = it->second;

//       boost::optional<gtsam::Pose3> cloud_pose = mapper_->predictPose (sym);

//       if (cloud_pose)
//       {
//         CloudPtr map_cloud (new Cloud ());
//         map_cloud->width = 640;
//         map_cloud->height = 480;
//         map_cloud->resize (map_cloud->width * map_cloud->height);

//         for (int i = 0; i < map_cloud->width * map_cloud->height; i++)
//         {
//           //  std::cout << i << std::endl;
//           PointT invalid_pt;
//           invalid_pt.x = std::numeric_limits<float>::quiet_NaN ();
//           invalid_pt.y = std::numeric_limits<float>::quiet_NaN ();
//           invalid_pt.z = std::numeric_limits<float>::quiet_NaN ();
//           invalid_pt.r = 1;
//           invalid_pt.g = 0;
//           invalid_pt.b = 0;
//           invalid_pt.a = 1;
//           map_cloud->points[i] = invalid_pt;

//         }

//         pcl::PointIndices clust_indices = indices.at (sym);

//         if(debug_)
//         std::cout << "[ObjectPlugin] Inside the loop: " << cloud->points.size ()
//             << " Size of clust indices " << clust_indices.indices.size ()
//             << std::endl;
//         //  std::cout << "Size of clust indices " << clust_indices.indices.size() << std::endl;

//         for (int i = 0; i < clust_indices.indices.size (); i++)
//         {
//           std::cout << "indices: " << clust_indices.indices[i] << std::endl;
//           map_cloud->points[clust_indices.indices[i]] = cloud->points[i];
//         }

//         gtsam::Pose3 sam_pose = *cloud_pose;
//         //const gtsam::Rot3 rot;
//         //  const gtsam::Point3 centroid_pt(obj_centroid[0], obj_centroid[1], obj_centroid[2]);
//         //gtsam::Pose3 centroid_tform(rot, centroid_pt);
//         //  gtsam::Pose3 inv_tform = centroid_tform.inverse();
//         //sam_pose = inv_tform*sam_pose; // order of multiplication should be kept in mind

//         Eigen::Matrix4f map_tform = sam_pose.matrix ().cast<float> ();
//         Eigen::Affine3d tform;
//         gtsam::Quaternion sam_quat = sam_pose.rotation ().toQuaternion ();
//         tform = Eigen::Quaterniond (sam_quat.w (), sam_quat.x (), sam_quat.y (),
//             sam_quat.z ());
//         tform.translation () = Eigen::Vector3d (sam_pose.x (), sam_pose.y (),
//             sam_pose.z ());

//         /* testing the transformation */
//         CloudPtr vis_cloud (new Cloud ());
//         pcl::transformPointCloud (*map_cloud, *vis_cloud, tform);
//         transformed_cloud_opt_ += *vis_cloud;

//         //Eigen::Affine3d tform_inv = tform.inverse();
//         //pcl::transformPointCloud (*frame_cloud, *map_cloud, map_tform);
//         //Eigen::Affine3d tform (Eigen::Quaterniond(sam_quat[0],sam_quat[1],sam_quat[2],sam_quat[3]), Eigen::Vector3d (sam_pose.x (), sam_pose.y (), sam_pose.z ()));

//         pcl::PointCloud<pcl::Normal> empty_normals;
//         pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
//         ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
//         ne.setMaxDepthChangeFactor (0.02f);
//         ne.setNormalSmoothingSize (20.0f);
//         ne.setDepthDependentSmoothing (true);
//         //ne.setRadiusSearch (0.1);
//         ne.setInputCloud (map_cloud);
//         ne.compute (empty_normals);
//         //empty_normals.resize (frame_cloud->points.size ());

//         if(debug_)
//         printf ("[ObjectPlugin] Cloud has: %d normals has: %d\n", cloud->points.size (),
//             empty_normals.points.size ());

//         if (cloud->points.size () > 0)
//         {
//           if(debug_)
//           std::cout << "[ObjectPlugin] Cloud integrated " << std::endl;
//           bool integration_flag = tsdf->integrateCloud<pcl::PointXYZRGBA,
//               pcl::Normal> (*map_cloud, empty_normals, tform);  // Integrate the cloud
//           if(debug_)
//           std::cout << "[ObjectPlugin] Integration done: " << integration_flag << std::endl;
//         }

//       }
//     }

//     /*
//      const gtsam::Rot3 rot;
//      const gtsam::Point3 centroid_pt(obj_centroid[0], obj_centroid[1], obj_centroid[2]);
//      gtsam::Pose3 centroid_tform(rot, centroid_pt);
//      gtsam::Pose3 inv_tform = centroid_tform.inverse();
//      Eigen::Matrix4f map_tform = inv_tform.matrix().cast<float>();

//      pcl::transformPointCloud(transformed_cloud_opt_, transformed_cloud_opt_,
//      map_tform);

//     std::string vis_file = object_database_location_+ "/test_models/"
//      */
//         + boost::lexical_cast<std::string> (id) + ".pcd";
//     pcl::io::savePCDFileASCII (vis_file, transformed_cloud_opt_);

//     std::string vol_file = object_database_location_ + "/tsdf_models/"
//         + boost::lexical_cast<std::string> (id) + ".vol";
//     tsdf->save (vol_file);  // Save it?

// // Maching Cubes
//     cpu_tsdf::MarchingCubesTSDFOctree mc;
//     mc.setInputTSDF (tsdf);
//     mc.setColorByConfidence (false);
//     mc.setColorByRGB (false);
//     mc.setMinWeight (0.001);
//     pcl::PolygonMesh mesh;
//     mc.reconstruct (mesh);

//     std::string output_file = object_database_location_ + "/tsdf_models/"
//         + boost::lexical_cast<std::string> (id) + ".ply";
//     pcl::io::savePLYFileBinary (output_file, mesh);

// // Render from xo
//     Eigen::Affine3d init_pose = Eigen::Affine3d::Identity ();
//     pcl::PointCloud<pcl::PointNormal>::Ptr raytraced = tsdf->renderView (
//         init_pose);
//     std::string render_file = object_database_location_ + "/tsdf_models/rendered_"
//         + boost::lexical_cast<std::string> (id) + ".pcd";

//     pcl::io::savePCDFileBinary (render_file, *raytraced);

//   }


}

template class omnimapper::Object<pcl::PointXYZRGBA>;
