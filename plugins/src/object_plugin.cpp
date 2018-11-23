#include <omnimapper/object_plugin.h>
//#include <omnimapper/plane.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/common/common.h>

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

namespace omnimapper
{
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT>
  ObjectPlugin<PointT>::ObjectPlugin (omnimapper::OmniMapperBase* mapper) :
    mapper_ (mapper),
    get_sensor_to_base_ (GetTransformFunctorPtr ()),
    observations_ (),
    empty_ (),
    max_object_size (0),
    max_current_size (0),
    debug_ (true),
    verbose_ (true),
    have_new_cloud_(false),
    min_cluster_height_ (0.3),
    max_cluster_dist_(3.0),
    max_bbox_volume_(0.5),
    max_bbox_dim_(1.0),
    min_curvature_(0.05),
    min_points_(5000),
    min_clust_centroid_ptp_dist(0.5),
    ptp_pt_cull_thresh_(0.02),
    filter_points_near_planes_(true),
  save_pcds_(false),
  object_pcd_location_(std::string("/home"))
  {
    printf ("In constructor, checking size of observations_\n");
    printf ("Size: %d\n", observations_.size ());


  }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT>
  ObjectPlugin<PointT>::~ObjectPlugin ()
  {
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT> void
  ObjectPlugin<PointT>::spin (){

    /** Process cloud in a thread */
    boost::thread process_cloud_thread (&ObjectPlugin<PointT>::processCloud, this);

    /** Refresh object models after every few seconds */
    //boost::thread update_thread (&ObjectPlugin<PointT>::refreshObjectModels, this);

  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  template<typename PointT> void ObjectPlugin<PointT>::setVisCallback (
                                                                       boost::function<
                                                                       void (std::map<gtsam::Symbol, ObjectPtr>)>& fn)
  {
    vis_callback_ = fn;    // set visualization call back to fn
    vis_flag_ = true;
  }


/**************************************//**
* TODO
******************************************/
template<typename PointT> void ObjectPlugin<PointT>::refreshObjectModels ()
{
    while (1)
    {
      {
        boost::mutex::scoped_lock lock(object_plugin_mutex_);

        if (verbose_)
          printf ("[ObjectPlugin] Creating Optimal Cloud\n");

        typename std::map<gtsam::Symbol, ObjectPtr>::iterator it;
        for (it = object_map_.begin (); it != object_map_.end (); it++)
        {
          ObjectPtr object = it->second;  // object
          object->updateOptimizedCloud();
          if(save_pcds_){
            std::string location(object_pcd_location_);
            object->saveAsPCD(location);
          }
        }

      }

      boost::this_thread::sleep (boost::posix_time::milliseconds (5000 )); // sleep for 1000ms

    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT>
  float ObjectPlugin<PointT>::computeBoundingBoxIntersection (
                                                              std::pair<Eigen::Vector4f, Eigen::Vector4f> bounding_box_a, std::pair<Eigen::Vector4f, Eigen::Vector4f> bounding_box_b)
  {
    Eigen::Vector4f min_pt_current = bounding_box_a.first;
    Eigen::Vector4f max_pt_current = bounding_box_a.second;

    /// compute intersection of boxes
    float minX = min_pt_current[0];
    float minY = min_pt_current[1];
    float minZ = min_pt_current[2];
    float maxX = max_pt_current[0];
    float maxY = max_pt_current[1];
    float maxZ = max_pt_current[2];


    Eigen::Vector4f min_pt_prev = bounding_box_b.first;
    Eigen::Vector4f max_pt_prev = bounding_box_b.second;

    float minX1 = min_pt_prev[0];
    float minY1 = min_pt_prev[1];
    float minZ1 = min_pt_prev[2];
    float maxX1 = max_pt_prev[0];
    float maxY1 = max_pt_prev[1];
    float maxZ1 = max_pt_prev[2];

    float inter_area = MAX(MIN(maxX,maxX1)-MAX(minX,minX1),0)
    * MAX(MIN(maxY,maxY1)-MAX(minY,minY1),0)
    * MAX(MIN(maxZ,maxZ1)-MAX(minZ,minZ1),0);

    float areaA = (maxX - minX) * (maxY - minY) * (maxZ - minZ);
    float areaB = (maxX1 - minX1) * (maxY1 - minY1) * (maxZ1 - minZ1);

    float jaccard_index = inter_area / (areaA + areaB - inter_area);

    if(debug_)
      printf("[ObjectPlugin] Jaccard Index: Area of intersection: %f, Area of A: %f, Area of B: %f\n",inter_area, areaA, areaB);

    return jaccard_index;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT>
  std::vector<gtsam::Symbol> ObjectPlugin<PointT>::linearMatch (CloudPtrVector label)
  {

    if(verbose_)
      printf("[ObjectPlugin] inside linearMatch...\n");

    centroid_vec_.clear();
    std::vector<std::pair<Eigen::Vector4f, Eigen::Vector4f> > bounding_box;
    for (int idx = 0; idx < label.size (); idx++)
    {
      Eigen::Vector4f xyz_centroid;
      CloudPtr cloud = label[idx];
      pcl::compute3DCentroid(*cloud, xyz_centroid);
      centroid_vec_.push_back(xyz_centroid);

      Eigen::Vector4f min_pt, max_pt;
      pcl::getMinMax3D(*cloud, min_pt, max_pt);
      bounding_box.push_back(std::make_pair(min_pt, max_pt));
    }

    int num_edges = centroid_vec_.size ();  // number of edges in bipartite matching graph

    std::vector<gtsam::Symbol> neighbor_vec;
    neighbor_vec.resize (num_edges, -1);
    typename std::map<gtsam::Symbol, ObjectPtr >::iterator object_it;

    for (size_t idx = 0; idx < centroid_vec_.size (); idx ++)
    {
      float min = FLT_MAX;
      gtsam::Symbol nearest_object = gtsam::symbol('n',1);
      for (object_it = object_map_.begin(); object_it != object_map_.end (); object_it++)
      {
        Eigen::Vector4f curr_centroid = centroid_vec_[idx];
        ObjectPtr object_ptr = object_it->second;
        gtsam::Symbol object_symbol = object_it->first;
        Eigen::Vector4f prev_centroid = object_ptr->centroid();
        if(debug_)
          std::cout << "[ObjectPlugin] Previous Centroid: " << prev_centroid
                    << " Current Centroid " << curr_centroid << std::endl;
        float diff_eq = (prev_centroid - curr_centroid).norm();

        if (diff_eq  < min)
        {
          min = diff_eq;
          nearest_object = object_symbol;
        }
      }

      if(gtsam::symbolChr(nearest_object) == 'n'){
        if(verbose_)
          printf("[ObjectPlugin] found a new object\n");
        neighbor_vec[idx] = gtsam::symbol('n',1);
      }
      else{
        neighbor_vec[idx] = nearest_object;
        ObjectPtr object = object_map_.at(nearest_object);
        std::pair<Eigen::Vector4f, Eigen::Vector4f> prev_bounding_box = object->boundingBox();

        // compute intersection of boxes
        float jaccard_index = computeBoundingBoxIntersection (bounding_box[idx], prev_bounding_box);

        std::cout << "jaccard_index: " << jaccard_index << std::endl;

        if (jaccard_index < 0.3 || min > 0.7)
        {
          std::cout << "Min: " << min << std::endl;
          std::cout << "jaccard_index: " << jaccard_index << std::endl;
          neighbor_vec[idx] = gtsam::symbol('n',1); ///> Didn't match anything, Probably a new object segment
        }
      }
    }

    if(verbose_)
      printf("[ObjectPlugin] done linearMatch...\n");

    return neighbor_vec;

  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT> void
  ObjectPlugin<PointT>::updateObjects(
                                      std::vector<gtsam::Symbol> neighbor_vec,
                                      std::map<gtsam::Symbol, std::vector<int> > rev_neighbor_vec,
                                      CloudPtrVector point_cloud,
                                      gtsam::Symbol cloud_symbol,
                                      gtsam::Pose3 cloud_pose)
  {

    std::vector<gtsam::Symbol> final_neighbor_vec;
    final_neighbor_vec.resize (neighbor_vec.size (), gtsam::symbol('n',1));

    for (size_t idx = 0; idx < neighbor_vec.size (); idx++)
    {
      /// Check if the segment matched to an object or not
      neighbor_vec[idx].print("Matching symbol: ");
      if (gtsam::symbolChr(neighbor_vec[idx])!='n')
      {
        gtsam::Symbol sym = neighbor_vec[idx];
        if (rev_neighbor_vec[sym].size () == 1)
        {
          final_neighbor_vec[idx] = sym;
          ObjectPtr object = object_map_.at(sym);
          if(verbose_)
            printf (
                    "[ObjectPlugin] Adding a new object with label %c%d and an object segment with size %d\n",
                    'o', sym.index(), point_cloud[idx]->points.size ());
          object->addObservation(cloud_symbol, point_cloud[idx], cloud_pose);
          continue;
        }

        // find the NN of i among rev_neighbor_vec[neighbor_vec[i]]
        float nn_nn_index = -1;
        float min = FLT_MAX;
        for (int j = 0; j < rev_neighbor_vec[sym].size (); j++)
        {

          int idy = rev_neighbor_vec[sym][j];
          Eigen::Vector4f curr_centroid = centroid_vec_[idy];
          ObjectPtr object = object_map_.at(sym);
          Eigen::Vector4f prev_centroid = object->centroid();
          Eigen::Vector4f diff = prev_centroid - curr_centroid;
          double diff_eq = diff.norm();

          if ( diff_eq < min)
          {
            min = diff_eq;
            nn_nn_index = idy;
          }
        }

        final_neighbor_vec[nn_nn_index] = sym;
        neighbor_vec[nn_nn_index] = sym;


        ObjectPtr object = object_map_.at(sym);
        if(verbose_)
          printf (
                  "[ObjectPlugin] Adding a new object with label %c%d and an object segment with size %d\n",
                  'o', sym.index(), point_cloud[nn_nn_index]->points.size ());
        object->addObservation(cloud_symbol, point_cloud[nn_nn_index], cloud_pose);


        /* for everything else assign a new label */
        for (int j = 0; j < rev_neighbor_vec[sym].size (); j++)
        {
          int idy = rev_neighbor_vec[sym][j];
          if (idy != nn_nn_index)
          {
            /// Find new label
            int new_label = object_map_.size() + 1;
            gtsam::Symbol O = gtsam::symbol('o',new_label);
            if(verbose_)
              printf (
                      "[ObjectPlugin] Adding a new object with label %c%d and an object segment with size %d\n",
                      'o', new_label, point_cloud[idy]->points.size ());

            omnimapper::Object <PointT>* new_object = new omnimapper::Object<PointT>(O);
            new_object->addObservation (cloud_symbol, point_cloud[idy], cloud_pose);
            ObjectPtr obj_ptr(new_object);
            object_map_.insert(std::pair<gtsam::Symbol, ObjectPtr> (O, obj_ptr));

          }
        }
        rev_neighbor_vec[sym].clear ();
        rev_neighbor_vec[sym].push_back (nn_nn_index);
      }
      else{

        /// Didn't match to anything
        int new_label = object_map_.size() + 1;
        gtsam::Symbol O = gtsam::symbol('o',new_label);
        omnimapper::Object <PointT>* new_object = new omnimapper::Object<PointT>(O);
        if(verbose_)
          printf (
                  "[ObjectPlugin] Adding a new object with label %c%d and an object segment with size %d\n",
                  'o', new_label, point_cloud[idx]->points.size ());
        new_object->addObservation (cloud_symbol, point_cloud[idx], cloud_pose);
        ObjectPtr obj_ptr(new_object);
        object_map_.insert(std::pair<gtsam::Symbol, ObjectPtr> (O, obj_ptr));
      }
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT>
  typename
  std::pair<std::vector<gtsam::Symbol>,
            std::map<gtsam::Symbol, std::vector<int> > >
  ObjectPlugin<PointT>::dataAssociate (CloudPtrVector label, gtsam::Pose3 pose, gtsam::Symbol sym)
  {

    if(verbose_)
      printf("[ObjectPlugin] dataAssociating...\n");

    double start = pcl::getTime ();

    /// Transform point clouds from robot frame to the map frame
    CloudPtrVector transformed_label;
    Eigen::Matrix4f map_tform = pose.matrix ().cast<float> ();
    for (int i = 0; i < label.size (); i++)
    {
      pcl::PointCloud<PointT> cluster;
      pcl::transformPointCloud (*label[i], cluster, map_tform);  ///> this is transforming the main point cloud
      transformed_label.push_back (cluster.makeShared ());
    }

    /// Compute centroid of the current frame and Match it to the previous frame
    std::vector<gtsam::Symbol> neighbor_vec = linearMatch (transformed_label);

    /// Do the reverse match
    std::map<gtsam::Symbol, std::vector<int> > rev_neighbor_vec;
    for (size_t idx = 0; idx < neighbor_vec.size (); idx++)
    {
      gtsam::Symbol sym = neighbor_vec[idx];
      if (rev_neighbor_vec.find (sym) != rev_neighbor_vec.end ())
      {
        rev_neighbor_vec[sym].push_back (idx);
      }
      else
      {
        std::vector<int> temp_nn_index;
        temp_nn_index.push_back (idx);
        rev_neighbor_vec[sym] = temp_nn_index;
      }
    }

    double end = pcl::getTime ();

    if(verbose_){
      printf("[ObjectPlugin] label propagation took %lf\n", double(end-start));
      printf("[ObjectPlugin] done dataAssociating...\n");
    }

    return std::make_pair(neighbor_vec, rev_neighbor_vec);

  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT> void
  ObjectPlugin<PointT>::processCloud ()
  {

    bool have_new_cloud;
    while(true){

      {
        boost::mutex::scoped_lock lock(object_plugin_mutex_);
        have_new_cloud = have_new_cloud_;
      }

      if(!have_new_cloud){
        boost::this_thread::sleep (boost::posix_time::milliseconds (10));
        if (debug_)
          printf("ObjectPlugin: waiting or new cloud....\n");
      }
      else{
        std::vector<CloudPtr> clusters = prev_clusters_;
        omnimapper::Time t = prev_time_;
        boost::optional<std::vector<pcl::PointIndices> > indices = prev_indices_;

        /** Get pose symbol for this timestamp */
        gtsam::Symbol pose_symbol;
        mapper_->getPoseSymbolAtTime (t, pose_symbol);
        boost::optional<gtsam::Pose3> cloud_pose = boost::none;
        while (!cloud_pose)
        {
          cloud_pose = mapper_->predictPose(pose_symbol);
          boost::this_thread::sleep(boost::posix_time::milliseconds(5));// sleep for a few ms
        }

        /** Transform point clouds to base_frame. All the sensors are mapped to the base frame and the corresponding pose
         *  is estimated using all the measurements from all the sensors  */
        std::vector<CloudPtr> clusters_base; ///> Push back the transformed cluster
        Eigen::Affine3d sensor_to_base = Eigen::Affine3d::Identity();
        if (get_sensor_to_base_)
        {
          if(debug_)
            printf ("[ObjectPlugin] Object Plugin: Applying sensor to base transform.\n");
          sensor_to_base = (*get_sensor_to_base_) (t);


          if(debug_)
            std::cout << "[ObjectPlugin] Sensor to Base Transform: " << sensor_to_base.matrix() << std::endl;
          for (int i = 0; i < clusters.size (); i++)
          {
            CloudPtr tformed_cluster (new Cloud ());
            pcl::transformPointCloud (*clusters[i], *tformed_cluster,
                                      sensor_to_base);
            clusters_base.push_back (tformed_cluster);
          }
        }
        else
        {
          clusters_base = clusters;
        }

        if(verbose_)
          printf ("[ObjectPlugin] Object plugin got %d clusters_base\n", clusters_base.size ());


        /** Keep track of indices using indices_base */
        std::vector<pcl::PointIndices> indices_base (clusters_base.size ());
        gtsam::Values solution;

        /*  temporarily removing planes
            if (filter_points_near_planes_)
            solution = mapper_->getSolution ();
            gtsam::Values::ConstFiltered<gtsam::Plane<PointT> > plane_filtered =
            solution.filter<gtsam::Plane<PointT> > ();
        */

        CloudPtrVector filtered_observations;
        std::vector<pcl::PointIndices> filtered_observation_indices;

        /** Compute cluster statistics for filtering purposes */
        Eigen::Matrix3f clust_cov;
        Eigen::Vector4f clust_centroid = Eigen::Vector4f::Zero ();
        Eigen::Vector4f min_pt;
        Eigen::Vector4f max_pt;
        EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
        EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;

        for (int i = 0; i < clusters_base.size (); i++)
        {
          /// Detect nearby planes
          /* temporarily removing planes
             if (filter_points_near_planes_)
             {
             /// See what plane (if any) is closest
             Eigen::Vector4f clust_centroid;
             pcl::compute3DCentroid ( (* (clusters_base[i])), clust_centroid);
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
             double ptp_dist = fabs (
             key_value.value.a () * clust_centroid[0]
             + key_value.value.b () * clust_centroid[1]
             + key_value.value.c () * clust_centroid[2]
             + key_value.value.d ());
             bool pt_in_poly = pcl::isPointIn2DPolygon (clust_centroid_pt,
             boundary_cloud);
             if (pt_in_poly && (ptp_dist < min_clust_centroid_ptp_dist))
             {
             found_nearby_plane = true;
             nearby_symbols.push_back (key_value.key);
             nearby_coeffs.push_back (
             Eigen::Vector4f (key_value.value.a (), key_value.value.b (),
             key_value.value.c (), key_value.value.d ()));
             }
             }

             // If we found a nearby plane, remove points that are very close to it
             if (found_nearby_plane)
             {
             //gtsam::Plane<PointT> plane = solution.at<gtsam::Plane<PointT> > (closest_sym);
             CloudPtr filtered_clust (new Cloud ());
             pcl::PointIndices filtered_indices;
             for (int j = 0; j < clusters_base[i]->points.size (); j++)
             {
             bool pt_ok = true;

             for (int k = 0; k < nearby_symbols.size (); k++)
             {
             double ptp_dist = fabs (
             nearby_coeffs[k][0] * clusters_base[i]->points[j].x
             + nearby_coeffs[k][1] * clusters_base[i]->points[j].y
             + nearby_coeffs[k][2] * clusters_base[i]->points[j].z
             + nearby_coeffs[k][3]);

             if (ptp_dist <= ptp_pt_cull_thresh_)
             pt_ok = false;

             // if (ptp_dist >= ptp_pt_cull_thresh_)
             // {
             //   filtered_clust->points.push_back (clusters_base[i]->points[j]);
             // }
             }

             if (pt_ok)
             {
             filtered_clust->points.push_back (clusters_base[i]->points[j]);
             filtered_indices.indices.push_back ( (*indices)[i].indices[j]);
             }
             }

             if(debug_)
             printf ("[ObjectPlugin] cluster %d had %d points, filtered has %d points\n", i,
             clusters_base[i]->points.size (), filtered_clust->points.size ());
             clusters_base[i] = filtered_clust;
             indices_base[i] = filtered_indices;

             if(debug_)
             printf ("[ObjectPlugin] filtered_indices: %d, indices_base: %d",
             filtered_indices.indices.size (),
             indices_base[i].indices.size ());

             if(debug_)
             printf ("[ObjectPlugin] cluster %d now has %d points\n", i,
             clusters_base[i]->points.size ());
             }
             else
             {
             pcl::PointIndices filtered_indices;
             for (int j = 0; j < clusters_base[i]->points.size (); j++)
             filtered_indices.indices.push_back ( (*indices)[i].indices[j]);

             indices_base[i] = filtered_indices;
             if(debug_)
             printf ("[ObjectPlugin] cluster has %d points, indices has %d points\n",
             clusters_base[i]->points.size (),
             indices_base[i].indices.size ());

             }
             }
          */

          // If we still have enough points, filter based on entire cluster
          if (clusters_base[i]->points.size () >= min_points_)
          {
            // Get mean and covariance matrix
            pcl::computeMeanAndCovarianceMatrix ( (* (clusters_base[i])), clust_cov,
                                                  clust_centroid);

            double dist = sqrt (
                                clust_centroid[0] * clust_centroid[0]
                                + clust_centroid[1] * clust_centroid[1]
                                + clust_centroid[2] * clust_centroid[2]);

            // Get bounding box
            pcl::getMinMax3D ( (* (clusters_base[i])), min_pt, max_pt);
            Eigen::Vector4f size = max_pt - min_pt;
            double bbox_volume = size[0] * size[1] * size[2];

            // Get curvature
            pcl::eigen33 (clust_cov, eigen_value, eigen_vector);
            double eig_sum = clust_cov.coeff (0) + clust_cov (4) + clust_cov (8);
            double curvature = 0;
            if (eig_sum != 0)
              curvature = fabsf (eigen_value / eig_sum);

            if(debug_){
              printf ("[ObjectPlugin]Cluster %d:\n", i);
              printf ("[ObjectPlugin]  Dist: %lf\n", dist);
              printf ("[ObjectPlugin]  Volume: %lf\n", bbox_volume);
              printf ("[ObjectPlugin]  Dims: %lf, %lf, %lf\n", size[0], size[1], size[2]);
              printf ("[ObjectPlugin]  Curvature: %lf\n", curvature);
            }
            // Clusters that are too far away suffer from too much depth discretization to be reliable
            bool dist_ok = dist < max_cluster_dist_;
            // Clusters that are too low in the robot frame can be excluded
            std::cout << "[ObjectPlugin] min_cluster_height " << min_cluster_height_ << " centroid " << clust_centroid[2] << "\n";
            bool height_ok = clust_centroid[2] > min_cluster_height_;
            // Clusters with large bounding boxes are probably not objects of interest -- these tend to architectural features
            bool bbox_volume_ok = bbox_volume < max_bbox_volume_;
            // Clusters that have some dimension that is quite large tend to be uninteresting
            bool bbox_dims_ok = (size[0] < max_bbox_dim_)
            && (size[1] < max_bbox_dim_) && (size[2] < max_bbox_dim_);
            // Clusters with too low curvature are probably either planar objects that were improperly segmented, or
            // Also, if it is mostly planar, shape descriptors will not be very reliable
            bool curvature_ok = curvature > min_curvature_;

            if(debug_)
              printf ("[ObjectPlugin] centroid dist: %lf\n", dist);
            if (dist_ok && height_ok && bbox_volume_ok && bbox_dims_ok
                && curvature_ok)
            {
              filtered_observations.push_back (clusters_base[i]);
              filtered_observation_indices.push_back (indices_base[i]);

              if(verbose_)
                std::cout << "[ObjectPlugin] Pushing cluster" << clusters_base[i]->points.size ()
                          << " and " << indices_base[i].indices.size () << std::endl;
            }
          }

        }

        if(debug_)
          printf("[ObjectPlugin] Size of filtered observations: %d\n", filtered_observations.size ());

        observations_.insert (
                              std::pair<gtsam::Symbol, CloudPtrVector> (pose_symbol,
                                                                        filtered_observations));

        observation_indices_.insert (
                                     std::pair<gtsam::Symbol, std::vector<pcl::PointIndices> > (pose_symbol,
                                                                                                filtered_observation_indices));

        if (cloud_pose && filtered_observations.size () != 0)
        {

          /** Associate each filtered observation with a set of Objects */
          double seg_start = pcl::getTime ();
          std::pair <std::vector<gtsam::Symbol>, std::map<gtsam::Symbol, std::vector<int> > > associations =
          dataAssociate (filtered_observations, *cloud_pose, pose_symbol);
          double seg_end = pcl::getTime ();

          updateObjects (associations.first, associations.second, filtered_observations,
                         pose_symbol, *cloud_pose);

          if(verbose_)
            printf("[ObjectPlugin] Segmentation took: %lf\n",double (seg_end - seg_start));

        }

        /** Visualize  */

        if (vis_flag_)
        {
          double vis_start = pcl::getTime ();
          if(verbose_)
            printf("[ObjectPlugin] Visualizing Objects\n");

          vis_callback_ (object_map_);
          double vis_end = pcl::getTime ();
          if(verbose_)
            printf("[ObjectPlugin] Done Visualizing Objects %lf\n",double (vis_end - vis_start));
        }

        {
          boost::mutex::scoped_lock lock(object_plugin_mutex_);
          have_new_cloud_ = false; ///< Done with the processing
        }

      }

    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT> void
  ObjectPlugin<PointT>::clusterCloudCallback (
                                              std::vector<CloudPtr> clusters,
                                              omnimapper::Time t,
                                              boost::optional<std::vector<pcl::PointIndices> > indices)
  {
    if(debug_)
      printf ("[ObjectPlugin] Got new Clusters: %d\n", clusters.size());

    if(!ready()){
      printf ("[ObjectPlugin] Sorry not yet ready\n");
      return; ///> keep dropping the cloud until its ready
    }

    if(debug_)
      printf ("[ObjectPlugin] Got the new cloud\n");

    {
      boost::mutex::scoped_lock lock(object_plugin_mutex_);
      prev_clusters_ = clusters;
      prev_time_ = t;
      prev_indices_ = indices;
      have_new_cloud_ = true; ///> Used by the evaluation plugin
    }
  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT> typename omnimapper::ObjectPlugin<PointT>::CloudPtrVector ObjectPlugin<
    PointT>::getObservations (gtsam::Symbol sym)
  {
    if(verbose_){
      printf ("[ObjectPlugin] Starting getObservations!\n");
      printf ("[ObjectPlugin] Checking size...\n");
    }

    if (observations_.size () == 0)
    {
      if(verbose_)
        printf ("[ObjectPlugin] returning empty due to size!\n");
      return (empty_);
    }

    if(verbose_)
      printf ("[ObjectPlugin] Checking count!\n");
    if (observations_.count (sym) > 0)
    {
      if(verbose_)
        printf ("[ObjectPlugin] ObjectPlugin::getObservations: returning vec!\n");
      return (observations_.at (sym));
    }
    else
    {
      if(verbose_)
        printf ("[ObjectPlugin] ObjectPlugin::getObservations: returning empty!\n");
      return (empty_);
    }
  }

}

// Instantiate
template class omnimapper::ObjectPlugin<pcl::PointXYZRGBA>;
