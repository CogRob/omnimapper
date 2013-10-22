#include <omnimapper/object_plugin.h>
#include <omnimapper/plane.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

namespace omnimapper
{
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT>
  ObjectPlugin<PointT>::ObjectPlugin (omnimapper::OmniMapperBase* mapper) :
      mapper_ (mapper), get_sensor_to_base_ (GetTransformFunctorPtr ()), observations_ (), empty_ (), max_object_size (
          0), max_current_size (0), tsdf (new cpu_tsdf::TSDFVolumeOctree), debug_(false), verbose_(false), do_loop_closures_(true)  {
    printf ("In constructor, checking size of observations_\n");
    printf ("Size: %d\n", observations_.size ());

    // thread to match objects
    boost::thread object_recognition_thread (
        &ObjectPlugin<PointT>::objectRecognitionLoop, this);

    //find optimal object models in a thread
    boost::thread reconstruction_thread (
        &ObjectPlugin<PointT>::computeOptimalObjectModel, this);

  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT>
  ObjectPlugin<PointT>::~ObjectPlugin ()
  {
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT> void ObjectPlugin<PointT>::setAndLoadObjectDatabaseLocation(std::string object_database_location){

    object_database_location_ = object_database_location; //location at which all the object pcds, descriptors are stored

    if(debug_)
    std::cout << "[ObjectPlugin] Object database location set: " << object_database_location_
        << std::endl;

    loadDatabase (); // load the database for object plugin
    if(debug_)
    std::cout << "[ObjectPlugin] Loaded representations " << std::endl;

    segment_propagation_.reset (new SegmentPropagation<PointT> ());
    segment_propagation_->setActiveLabelIndices (max_object_size); // initialize segment propagation
    if(debug_)
    std::cout << "[ObjectPlugin] Active label indices set" << std::endl;

    object_discovery_.reset (new ObjectDiscovery<PointT> ());
    object_discovery_->loadRepresentations(object_database_location_); //load the database for object discovery thread
    // boost::thread object_discovery_thread (&ObjectPlugin<PointT>::objectDiscoveryLoop, this);
     }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT> void ObjectPlugin<PointT>::setObjectCallback (
      boost::function<
          void (std::map<gtsam::Symbol, Object<PointT> >, gtsam::Point3, gtsam::Point3)>& fn)
  {
    vis_callback_ = fn;    // set visualization call back to fn
    vis_flag_ = true;
  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT>
  void ObjectPlugin<PointT>::loadDatabase ()
  {
    if(debug_)
    std::cout << "[ObjectPlugin] Inside loadDesc" << std::endl;
    pcl::SIFTKeypoint<PointT, pcl::PointXYZI>* sift3D = new pcl::SIFTKeypoint<
        PointT, pcl::PointXYZI>;
    sift3D->setScales (0.01f, 3, 2);
    sift3D->setMinimumContrast (0.0);

    //pcl::UniformSampling<PointT>* uniform_sampling_ = new pcl::UniformSampling<PointT>;
    //uniform_sampling_->setRadiusSearch(0.01f);

    boost::shared_ptr<pcl::Keypoint<PointT, pcl::PointXYZI> > keypoint_detector;
    keypoint_detector.reset (sift3D);

    /*
     typename pcl::Feature<PointT, pcl::VFHSignature308>::Ptr feature_extractor (new pcl::CVFHEstimation<PointT, pcl::Normal, pcl::VFHSignature308>);
     feature_extractor->setSearchMethod (typename pcl::search::Search<PointT>::Ptr (new pcl::search::KdTree<PointT>));
     //feature_extractor->setRadiusSearch (0.05);
     typename pcl::PointCloud<pcl::VFHSignature308>::Ptr features(new pcl::PointCloud<pcl::VFHSignature308>);
     */
    /* and descriptors */

    // PFHRGB
    /*
     typename pcl::Feature<PointT, pcl::PFHRGBSignature250>::Ptr feature_extractor (new pcl::PFHRGBEstimation<PointT, pcl::Normal, pcl::PFHRGBSignature250>);
     feature_extractor->setSearchMethod (typename pcl::search::Search<PointT>::Ptr (new pcl::search::KdTree<PointT>));
     feature_extractor->setRadiusSearch (0.05);
     typename pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr features(new pcl::PointCloud<pcl::PFHRGBSignature250>);
     */

    // FPFH
    /*
     pcl::Feature<PointT, pcl::FPFHSignature33>::Ptr feature_extractor (new pcl::FPFHEstimationOMP<PointT, pcl::Normal, pcl::FPFHSignature33>);
     feature_extractor->setSearchMethod (pcl::search::Search<PointT>::Ptr (new pcl::search::KdTree<PointT>));
     feature_extractor->setRadiusSearch (0.05);
     pcl::PointCloud<pcl::FPFHSignature33>::Ptr features(new pcl::PointCloud<pcl::FPFHSignature33>);
     */

    // SHOT
    pcl::SHOTColorEstimationOMP<PointT, pcl::Normal, pcl::SHOT1344>* shot =
        new pcl::SHOTColorEstimationOMP<PointT, pcl::Normal, pcl::SHOT1344>;
    shot->setRadiusSearch (0.1);
    typename pcl::Feature<PointT, pcl::SHOT1344>::Ptr feature_extractor (shot);
    pcl::PointCloud<pcl::SHOT1344>::Ptr features (
        new pcl::PointCloud<pcl::SHOT1344>);

    /* initialize correspondence estimator */
    //object_recognition_.reset(new ObjectRecognition<pcl::FPFHSignature33>(keypoint_detector, feature_extractor));
    object_recognition_.reset (
        new ObjectRecognition<pcl::SHOT1344> (keypoint_detector,
            feature_extractor));

    if(debug_)
    std::cout << "[ObjectPlugin] Descriptor Loaded " << std::endl;
    /* load object descriptors */

    if(verbose_)
    std::cout << "[ObjectPlugin] Object location: " << object_database_location_ << std::endl;
    int max_segment = object_recognition_->loadDatabase (object_database_location_); //home/siddharth/kinect

    max_object_size = max_segment + 1;
    max_current_size = max_object_size;

    if(debug_)
    std::cout << "[ObjectPlugin] Size of max_segment " << max_segment + 1 << std::endl;

  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  template<typename PointT>
  void ObjectPlugin<PointT>::reconstructSurface (CloudPtr merged, int id)
  {
    if(verbose_)
    std::cout << "[ObjectPlugin] surface reconstruction..." << std::flush;

    // apply grid filtering to reduce amount of points as well as to make them uniform distributed
    pcl::VoxelGrid<pcl::PointXYZRGBA> voxel_grid;
    voxel_grid.setInputCloud (merged);
    voxel_grid.setLeafSize (0.002f, 0.002f, 0.002f);
    voxel_grid.setDownsampleAllData (true);
    voxel_grid.filter (*merged);

    pcl::PointCloud<pcl::Normal>::Ptr vertices (
        new pcl::PointCloud<pcl::Normal>);
    pcl::copyPointCloud (*merged, *vertices);

    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> normal_estimation;
    normal_estimation.setSearchMethod (
        pcl::search::Search<pcl::PointXYZRGBA>::Ptr (
            new pcl::search::KdTree<pcl::PointXYZRGBA>));
    normal_estimation.setRadiusSearch (0.01);
    normal_estimation.setInputCloud (merged);
    normal_estimation.compute (*vertices);

    pcl::search::KdTree<pcl::Normal>::Ptr tree (
        new pcl::search::KdTree<pcl::Normal>);
    tree->setInputCloud (vertices);

    pcl::PolygonMesh surface_;
    boost::shared_ptr<pcl::PCLSurfaceBase<pcl::Normal> > surface_reconstructor_;
    surface_reconstructor_->setSearchMethod (tree);
    surface_reconstructor_->setInputCloud (vertices);
    surface_reconstructor_->reconstruct (surface_);

    if(verbose_)
    std::cout << "OK" << std::endl;

    std::string output_file = object_database_location_ + "/tsdf_models/"
        + boost::lexical_cast<std::string> (id) + ".ply";
    pcl::io::savePLYFileBinary (output_file, surface_);

  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  template<typename PointT>
  void ObjectPlugin<PointT>::computeTSDF (Object<PointT> object,
      Eigen::Vector4f obj_centroid)
  {
    if(verbose_)
    printf ("[ObjectPlugin] starting generateTSDF\n");

    gtsam::Symbol object_symbol = object.sym;
    int id = object_symbol.index();

// Make a TSDF

    tsdf->setGridSize (10.0, 10.0, 10.0);
    tsdf->setResolution (2048, 2048, 2048);

    Eigen::Affine3d object_center;
    object_center.translation () = Eigen::Vector3d (obj_centroid[0],
        obj_centroid[1], obj_centroid[2]);
    Eigen::Affine3d tsdf_center = Eigen::Affine3d::Identity ();  // Optionally offset the center
    tsdf->setGlobalTransform (object_center);
//tsdf->setDepthTruncationLimits ();
//tsdf->setDepthTruncationLimits (0.3, 10.0);
//tsdf->setWeightTruncationLimit (100.0);
    tsdf->reset ();  // Initialize it to be empty

    std::map<gtsam::Symbol, CloudPtr> cluster = object.clusters_;
    std::map<gtsam::Symbol, pcl::PointIndices> indices = object.indices_;

    typename std::map<gtsam::Symbol, CloudPtr>::iterator it;

    Cloud transformed_cloud_opt_;

    for (it = cluster.begin (); it != cluster.end (); it++)
    {

      gtsam::Symbol sym = it->first;
      CloudPtr cloud = it->second;

      boost::optional<gtsam::Pose3> cloud_pose = mapper_->predictPose (sym);

      if (cloud_pose)
      {
        CloudPtr map_cloud (new Cloud ());
        map_cloud->width = 640;
        map_cloud->height = 480;
        map_cloud->resize (map_cloud->width * map_cloud->height);

        for (int i = 0; i < map_cloud->width * map_cloud->height; i++)
        {
          //	std::cout << i << std::endl;
          PointT invalid_pt;
          invalid_pt.x = std::numeric_limits<float>::quiet_NaN ();
          invalid_pt.y = std::numeric_limits<float>::quiet_NaN ();
          invalid_pt.z = std::numeric_limits<float>::quiet_NaN ();
          invalid_pt.r = 1;
          invalid_pt.g = 0;
          invalid_pt.b = 0;
          invalid_pt.a = 1;
          map_cloud->points[i] = invalid_pt;

        }

        pcl::PointIndices clust_indices = indices.at (sym);

        if(debug_)
        std::cout << "[ObjectPlugin] Inside the loop: " << cloud->points.size ()
            << " Size of clust indices " << clust_indices.indices.size ()
            << std::endl;
        //	std::cout << "Size of clust indices " << clust_indices.indices.size() << std::endl;

        for (int i = 0; i < clust_indices.indices.size (); i++)
        {
          std::cout << "indices: " << clust_indices.indices[i] << std::endl;
          map_cloud->points[clust_indices.indices[i]] = cloud->points[i];
        }

        gtsam::Pose3 sam_pose = *cloud_pose;
        //const gtsam::Rot3 rot;
        //	const gtsam::Point3 centroid_pt(obj_centroid[0], obj_centroid[1], obj_centroid[2]);
        //gtsam::Pose3 centroid_tform(rot, centroid_pt);
        //	gtsam::Pose3 inv_tform = centroid_tform.inverse();
        //sam_pose = inv_tform*sam_pose; // order of multiplication should be kept in mind

        Eigen::Matrix4f map_tform = sam_pose.matrix ().cast<float> ();
        Eigen::Affine3d tform;
        gtsam::Quaternion sam_quat = sam_pose.rotation ().toQuaternion ();
        tform = Eigen::Quaterniond (sam_quat.w (), sam_quat.x (), sam_quat.y (),
            sam_quat.z ());
        tform.translation () = Eigen::Vector3d (sam_pose.x (), sam_pose.y (),
            sam_pose.z ());

        /* testing the transformation */
        CloudPtr vis_cloud (new Cloud ());
        pcl::transformPointCloud (*map_cloud, *vis_cloud, tform);
        transformed_cloud_opt_ += *vis_cloud;

        //Eigen::Affine3d tform_inv = tform.inverse();
        //pcl::transformPointCloud (*frame_cloud, *map_cloud, map_tform);
        //Eigen::Affine3d tform (Eigen::Quaterniond(sam_quat[0],sam_quat[1],sam_quat[2],sam_quat[3]), Eigen::Vector3d (sam_pose.x (), sam_pose.y (), sam_pose.z ()));

        pcl::PointCloud<pcl::Normal> empty_normals;
        pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
        ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
        ne.setMaxDepthChangeFactor (0.02f);
        ne.setNormalSmoothingSize (20.0f);
        ne.setDepthDependentSmoothing (true);
        //ne.setRadiusSearch (0.1);
        ne.setInputCloud (map_cloud);
        ne.compute (empty_normals);
        //empty_normals.resize (frame_cloud->points.size ());

        if(debug_)
        printf ("[ObjectPlugin] Cloud has: %d normals has: %d\n", cloud->points.size (),
            empty_normals.points.size ());

        if (cloud->points.size () > 0)
        {
          if(debug_)
          std::cout << "[ObjectPlugin] Cloud integrated " << std::endl;
          bool integration_flag = tsdf->integrateCloud<pcl::PointXYZRGBA,
              pcl::Normal> (*map_cloud, empty_normals, tform);  // Integrate the cloud
          if(debug_)
          std::cout << "[ObjectPlugin] Integration done: " << integration_flag << std::endl;
        }

      }
    }

    /*
     const gtsam::Rot3 rot;
     const gtsam::Point3 centroid_pt(obj_centroid[0], obj_centroid[1], obj_centroid[2]);
     gtsam::Pose3 centroid_tform(rot, centroid_pt);
     gtsam::Pose3 inv_tform = centroid_tform.inverse();
     Eigen::Matrix4f map_tform = inv_tform.matrix().cast<float>();

     pcl::transformPointCloud(transformed_cloud_opt_, transformed_cloud_opt_,
     map_tform);
     */

    std::string vis_file = object_database_location_+ "/test_models/"
        + boost::lexical_cast<std::string> (id) + ".pcd";
    pcl::io::savePCDFileASCII (vis_file, transformed_cloud_opt_);

    std::string vol_file = object_database_location_ + "/tsdf_models/"
        + boost::lexical_cast<std::string> (id) + ".vol";
    tsdf->save (vol_file);  // Save it?

// Maching Cubes
    cpu_tsdf::MarchingCubesTSDFOctree mc;
    mc.setInputTSDF (tsdf);
    mc.setColorByConfidence (false);
    mc.setColorByRGB (false);
    mc.setMinWeight (0.001);
    pcl::PolygonMesh mesh;
    mc.reconstruct (mesh);

    std::string output_file = object_database_location_ + "/tsdf_models/"
        + boost::lexical_cast<std::string> (id) + ".ply";
    pcl::io::savePLYFileBinary (output_file, mesh);

// Render from xo
    Eigen::Affine3d init_pose = Eigen::Affine3d::Identity ();
    pcl::PointCloud<pcl::PointNormal>::Ptr raytraced = tsdf->renderView (
        init_pose);
    std::string render_file = object_database_location_ + "/tsdf_models/rendered_"
        + boost::lexical_cast<std::string> (id) + ".pcd";

    pcl::io::savePCDFileBinary (render_file, *raytraced);

  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT>
  void ObjectPlugin<PointT>::computeOptimalObjectModel ()
  {
    while(1)
    {
      if(verbose_)
    std::cout << "[ObjectPlugin] Creating Optimal Cloud\n" << std::endl;
    typename std::map<gtsam::Symbol, Object<PointT> >::iterator it;
    typename std::map<gtsam::Symbol, CloudPtr>::iterator it_cluster;

    map_building_mutex_.lock(); //lock the object map
    int map_size = object_map.size();
    int obj_count = 0;
    map_building_mutex_.unlock(); //unlock the object map

    for (it = object_map.begin (); it != object_map.end (); it++, obj_count++)
    {
      if(obj_count >= map_size)break; //making it thread safe, TODO: find a better way to make object plugin thread safe

      gtsam::Symbol sym = it->first;  //object symbol
      Object<PointT>& object = it->second;  // object

      // lock the object so that none of the new segments are added
      object.object_mutex_.lock();

      std::map<gtsam::Symbol, CloudPtr> cluster = it->second.clusters_;
      Cloud transformed_cloud_opt_;

      for (it_cluster = cluster.begin (); it_cluster != cluster.end ();
          it_cluster++)
      {

        gtsam::Symbol pose_sym = it_cluster->first; // robot pose symbol
        CloudPtr cloud = it_cluster->second; // cloud pointer

        boost::optional<gtsam::Pose3> cloud_pose = mapper_->predictPose (
            pose_sym); // get the optimized pose
        if (cloud_pose)
        {
          CloudPtr map_cloud (new Cloud ());
          gtsam::Pose3 new_pose = *cloud_pose;
          Eigen::Matrix4f map_transform = new_pose.matrix ().cast<float> ();
          pcl::transformPointCloud (*cloud, *map_cloud, map_transform); // transform cloud from base_frame to world_frame
          transformed_cloud_opt_ = transformed_cloud_opt_ + *map_cloud;

        }
      }

      object.optimal_cloud_ = transformed_cloud_opt_.makeShared(); // fill in the optimal cloud
      object.object_mutex_.unlock(); //unlock the object
    }


    boost::this_thread::sleep (boost::posix_time::milliseconds (100 )); // sleep for 1000ms
    }

  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  template<typename PointT>
  void ObjectPlugin<PointT>::recognizeObject (Object<PointT>& object)
  {

    int id = object.sym.index(); //object id
    std::map<gtsam::Symbol, CloudPtr> cluster = object.clusters_; //cloud ptr



    if(debug_)
    std::cout << "[ObjectPlugin] Object " << id << " seen in " << cluster.size () << " frames"
        << std::endl;

    std::vector<gtsam::Pose3> pose_array;
    typename std::map<gtsam::Symbol, CloudPtr>::iterator it;

    Cloud transformed_cloud_opt_;
    gtsam::Symbol obj_symbol = object.sym;
    gtsam::SharedDiagonal measurement_noise;
    measurement_noise = gtsam::noiseModel::Diagonal::Sigmas (
        gtsam::Vector_ (3, 0.5, 0.5, 0.5));


    for (it = cluster.begin (); it != cluster.end (); it++)
    {

      gtsam::Symbol sym = it->first;
      CloudPtr cloud = it->second;

      boost::optional<gtsam::Pose3> cloud_pose = mapper_->predictPose (sym);
      if (cloud_pose)
      {
        CloudPtr map_cloud (new Cloud ());
        gtsam::Pose3 new_pose = *cloud_pose;
        Eigen::Matrix4f map_transform = new_pose.matrix ().cast<float> ();
        pcl::transformPointCloud (*cloud, *map_cloud, map_transform);
        transformed_cloud_opt_ = transformed_cloud_opt_ + *map_cloud;
        pose_array.push_back (new_pose);

      }
    }

    Eigen::Vector4f obj_centroid;
    pcl::compute3DCentroid (transformed_cloud_opt_, obj_centroid);

    if(debug_)
    std::cout << "[ObjectPlugin] Object " << id << " centroid is " << obj_centroid[0] << " "
        << obj_centroid[1] << " " << obj_centroid[2] << std::endl;

    gtsam::Point3 object_centroid_pt (obj_centroid[0], obj_centroid[1],
        obj_centroid[2]); //object centroid

    /* check if object already exists in the map otherwise add it */
    if (!mapper_->getSolution ().exists (obj_symbol) && !object.landmark)
    {
      if(verbose_)
      obj_symbol.print ("[ObjectPlugin] Added object");
      mapper_->addNewValue (obj_symbol, object_centroid_pt);
      object.landmark = true;
    }

    /* add individual constraints between robot poses and object */
    for (it = cluster.begin (); it != cluster.end (); it++)
    {
      gtsam::Symbol sym = it->first;
      CloudPtr cloud = it->second;

      boost::optional<gtsam::Pose3> cloud_pose = mapper_->predictPose (sym);
      if (cloud_pose)
      {

        gtsam::Pose3 new_pose = *cloud_pose;
        Eigen::Vector4f measurement_centroid;
        pcl::compute3DCentroid (*cloud, measurement_centroid);
        const gtsam::Point3 measurement_pt (measurement_centroid[0],
            measurement_centroid[1], measurement_centroid[2]); //centroid in the robot base frame

        // check if the factor is already added
        if (object.factor_flag.at (sym) == -1)
        {
          object.factor_flag.at (sym) = 1;
        }
        else
        {
          continue;
        }

        // object centroid Point3 is tranformed in the robot pose Pose3 and the error is computed w.r.t measurement pt
        omnimapper::OmniMapperBase::NonlinearFactorPtr object_factor (
            new gtsam::LandmarkTransformFactor<gtsam::Pose3, gtsam::Point3> (
                sym, obj_symbol, measurement_pt, measurement_noise));
        mapper_->addFactor (object_factor);

        if(verbose_)
        obj_symbol.print ("[ObjectPlugin] Added factor for ");
      }
    }

    /*
     * Compute marginals of existing objects
     */

    gtsam::Matrix3 covariance_mat = gtsam::zeros (3, 3);
    int covSize = 0;

    gtsam::NonlinearFactorGraph graph = mapper_->getGraph ();
    gtsam::Values solution = mapper_->getSolution ();
    gtsam::Marginals marginals (graph, solution);
    if(debug_)
    graph.print ("[ObjectPlugin] Objects\n");
    if (solution.exists (obj_symbol))
    {
      try
      {
        if(debug_)
        obj_symbol.print ("[ObjectPlugin] Finding marginal covariance for");
        gtsam::Matrix mat = marginals.marginalCovariance (obj_symbol);
        if(debug_)
        std::cout << "[ObjectPlugin] Marginal computed" << std::endl;
        covariance_mat += mat;
      }
      catch (std::out_of_range)
      {
        if(debug_){
        std::cout << "[ObjectPlugin] Out of range: " << std::endl;
        std::cout << "[ObjectPlugin] Do solution exists: " << solution.exists (obj_symbol)
            << std::endl;
        }
        //	graph.print("Out of range error:");

        covariance_mat (0, 0) = 0.001;
        covariance_mat (1, 1) = 0.001;
        covariance_mat (2, 2) = 0.001;
        //	return;
      }

    }
    else
    {
      covariance_mat (0, 0) = 0.001;
      covariance_mat (1, 1) = 0.001;
      covariance_mat (2, 2) = 0.001;
      if(debug_)
      std::cout << "[ObjectPlugin] Covariance is " << covariance_mat << std::endl;
      //	return;
    }

    /* Surface reconstruction
     *
     //   reconstructSurface(transformed_cloud_opt_.makeShared(), id);
     const gtsam::Rot3 rot;
     const gtsam::Point3 centroid_pt(obj_centroid[0], obj_centroid[1], obj_centroid[2]);
     gtsam::Pose3 centroid_tform(rot, centroid_pt);
     gtsam::Pose3 inv_tform = centroid_tform.inverse();
     Eigen::Matrix4f map_tform = inv_tform.matrix().cast<float>();

     pcl::transformPointCloud(transformed_cloud_opt_, transformed_cloud_opt_,
     map_tform);

         //std::cout << "PCD file saved"
    //computeTSDF(object, obj_centroid);

     */

    /* Save PCDs to disk */

    if(verbose_)
    std::cout << "[ObjectPlugin] Saving new model" << std::endl;
    std::string output_file = object_database_location_ + "/object_models/"
        + boost::lexical_cast<std::string> (id) + ".pcd";

    if(transformed_cloud_opt_.points.size()>0)
    pcl::io::savePCDFileASCII (output_file, transformed_cloud_opt_);

    if(verbose_)
    std::cout << "[ObjectPlugin] Model saved" << std::endl;

    /* match the object to the database */
    std::pair<int, int> obj = object_recognition_->matchToDatabase (
        transformed_cloud_opt_.makeShared (), pose_array, id, covariance_mat);


    if (obj.first == -2)
    {
      if(verbose_)
      std::cout << "[ObjectPlugin] Didn't find a lot of features " << std::endl;

    }
    else if (obj.first == -1)
    {
      if(verbose_)
      std::cout << "[ObjectPlugin] Didn't match to anything" << std::endl;

    }
    else
    {
      if(debug_)
      std::cout << "[ObjectPlugin] Object " << id << " matched to " << obj.first << std::endl;

      if(verbose_)
      std::cout << "[ObjectPlugin] Matched with the same label" << std::endl;


      if (obj.first < max_object_size)
      {

        // matches to a saved representation
        Cloud matched_cloud;
        std::string input_file = object_database_location_ + "/object_models/"
            + boost::lexical_cast<std::string> (obj.first) + ".pcd";
        pcl::io::loadPCDFile<PointT> (input_file, matched_cloud);

        Eigen::Vector4f matched_centroid;
        pcl::compute3DCentroid (matched_cloud, matched_centroid);
        gtsam::Point3 matched_centroid_pt (matched_centroid[0],
            matched_centroid[1], matched_centroid[2]);

        gtsam::Point3 zero_pt (0, 0, 0);
        gtsam::Symbol object_symbol = object.sym;
        gtsam::SharedDiagonal measurement_noise;
        measurement_noise = gtsam::noiseModel::Diagonal::Sigmas (
            gtsam::Vector_ (3, 5.5, 5.5, 5.5));

        gtsam::Symbol match_symbol = gtsam::Symbol ('o', obj.first);  // matching object symbol

        if (!mapper_->getSolution ().exists (match_symbol))
        {
          mapper_->addNewValue (match_symbol, matched_centroid_pt);
        }

        // add a between factor between the two objects
        omnimapper::OmniMapperBase::NonlinearFactorPtr object_object_factor (
            new gtsam::BetweenFactor<gtsam::Point3> (obj_symbol, match_symbol,
                zero_pt, measurement_noise));
        mapper_->addFactor (object_object_factor);

      }
      else if (obj.first >= max_object_size && obj.first != id)  // check if the matching object is the same object
      {

        // loop closure detection
        if (do_loop_closures_)
        {


          /* load the matching object */
          gtsam::Symbol match_symbol = gtsam::Symbol ('o', obj.first);
          Cloud matched_cloud = object_map.at(match_symbol).optimalCloud();

          /* compute the centroid */
          Eigen::Vector4f matched_centroid;
          pcl::compute3DCentroid (matched_cloud, matched_centroid);
          gtsam::Point3 matched_centroid_pt (matched_centroid[0],
              matched_centroid[1], matched_centroid[2]);
          gtsam::Point3 zero_pt (0, 0, 0);

          gtsam::Symbol object_symbol = object.sym;
          gtsam::SharedDiagonal measurement_noise;
          measurement_noise = gtsam::noiseModel::Diagonal::Sigmas (
              gtsam::Vector_ (3, 0.5, 0.5, 0.5));

          if(verbose_)
          match_symbol.print ("[ObjectPlugin] object matched\n");

          if (!mapper_->getSolution ().exists (match_symbol))
          {
            if(verbose_)
            std::cout << "[ObjectPlugin] Adding new value" << std::endl;
            mapper_->addNewValue (match_symbol, matched_centroid_pt);
          }

          // add between factor
          if(verbose_)
          std::cout << "[ObjectPlugin] Adding inbetween factor" << std::endl;
          omnimapper::OmniMapperBase::NonlinearFactorPtr object_object_factor (
              new gtsam::BetweenFactor<gtsam::Point3> (obj_symbol, match_symbol,
                  zero_pt, measurement_noise));
          mapper_->addFactor (object_object_factor);

        }

      }

    }

    // store the camera poses with the object representation
    //matchDesc(transformed_cloud_opt_);
    object_recognition_->saveMapping (
       object_database_location_ + "/mapping.txt");
    object_recognition_->saveObjectStats (
        object_database_location_ + "/stats.txt");

    return;

  }

  template<typename PointT>
  gtsam::Symbol ObjectPlugin<PointT>::popFromQueue ()
  {
    /* push into recognition queue */
    boost::mutex::scoped_lock (recog_mutex_);
    gtsam::Symbol sym = train_queue.front ();
    train_queue.pop ();

    return sym;
  }

  template<typename PointT>
  void ObjectPlugin<PointT>::pushIntoQueue (gtsam::Symbol sym)
  {
    /* pop out of recognition queue */
    boost::mutex::scoped_lock (recog_mutex_);
    train_queue.push (sym);
    return;
  }

  template<typename PointT>
  void ObjectPlugin<PointT>::objectRecognitionLoop ()
  {

    std::map<gtsam::Symbol, int>::iterator it;
    while (1)
    {
      while (!train_queue.empty ())
      {

        gtsam::Symbol sym = popFromQueue ();

        if(verbose_)
        sym.print ("[ObjectPlugin] Training Object: ");
        if(debug_){
        std::cout << "[ObjectPlugin] Symbol index: " << sym.index () << std::endl;
        std::cout << "[ObjectPlugin] #Objects in Queue: " << train_queue.size () << std::endl;
        }

        recognizeObject (object_map.at (sym));

      }
      boost::this_thread::sleep (boost::posix_time::milliseconds (10)); // sleep for 10ms

    }

  }

  template<typename PointT>
  void ObjectPlugin<PointT>::objectDiscoveryLoop ()
  {
    // takes in all objects and undersegmented object parts and merge them using connected component to form full objects */
    // TODO: use the result of object discovery and update the object map for recognition

    std::string discover_dir = object_database_location_ +"/object_models";
    if(verbose_)
    std::cout << "[ObjectPlugin] Discovering objects " << std::endl;
    object_discovery_->createFinalCloud (discover_dir);
    object_discovery_->createGraph (); //match graph
    object_discovery_->mergeClouds ();
    boost::this_thread::sleep (boost::posix_time::milliseconds (10)); //sleep for 10 ms

  }

  template<typename PointT> float ObjectPlugin<PointT>::computeIntersection (
      Eigen::Vector4f minA, Eigen::Vector4f maxA, Eigen::Vector4f minB,
      Eigen::Vector4f maxB)
  {

/* compute bounding box */
    float minX = minA[0];
    float minY = minA[1];
    float minZ = minA[2];
    float maxX = maxA[0];
    float maxY = maxA[1];
    float maxZ = maxA[2];

    float minX1 = minB[0];
    float minY1 = minB[1];
    float minZ1 = minB[2];
    float maxX1 = maxB[0];
    float maxY1 = maxB[1];
    float maxZ1 = maxB[2];


    /* area of intersection */
    float inter_area = MAX(MIN(maxX,maxX1)-MAX(minX,minX1),0)
        * MAX(MIN(maxY,maxY1)-MAX(minY,minY1),0)
        * MAX(MIN(maxZ,maxZ1)-MAX(minZ,minZ1),0);

    float areaA = (maxX - minX) * (maxY - minY) * (maxZ - minZ);
    float areaB = (maxX1 - minX1) * (maxY1 - minY1) * (maxZ1 - minZ1);

/* jaccard index = (A \int B)/(A \union B) */
    float jaccard_index = inter_area / (areaA + areaB - inter_area);

    return jaccard_index;
  }

  template<typename PointT> float ObjectPlugin<PointT>::computeViewIntersection (
      gtsam::Point3 view_direction, gtsam::Point3 view_center, Eigen::Vector4f obj_centroid)
    {

        /* compute object direction */

    gtsam::Point3 obj_center (obj_centroid[0], obj_centroid[1],
        obj_centroid[2]);  //center of the object in world frame
    gtsam::Point3 obj_vector = obj_center-view_center; //camera center to object center direction
    double norm=obj_vector.norm(); //direction norm
    gtsam::Point3 obj_direction (obj_vector.x () / norm, obj_vector.y () / norm,
        obj_vector.z () / norm); //normalized direction

     /* angle between object and camera view */
    double angle = acos((view_direction.dot(obj_direction)))*(180/M_PI);

    if(debug_)
    std::cout << "[ObjectPlugin] [computeViewIntersection] object angle: " << angle << std::endl;

    /* distance between object and camera view */
    double distance = norm;

    if (norm > 4 || angle > 60)  // object is either more than 4 meters away or not in the view frustum
      return 0;
    else
      return 1;

    }
 //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT> void ObjectPlugin<PointT>::clusterCloudCallback (
      std::vector<CloudPtr> clusters, omnimapper::Time t,
      boost::optional<std::vector<pcl::PointIndices> > indices)
  {
    double max_cluster_dist_ = 3.0;
    double max_bbox_volume_ = 1.0;
    double max_bbox_dim_ = 1.0;
    double min_curvature_ = 0.01;
    double min_points_ = 1000;
    double min_clust_centroid_ptp_dist = 0.5;
    double ptp_pt_cull_thresh_ = 0.02;
    double min_cluster_height_ = 0.3;

    bool filter_points_near_planes_ = true;

    std::vector<CloudPtr> clusters_base;

    // Get pose symbol for this timestamp
     gtsam::Symbol pose_symbol;
     mapper_->getPoseSymbolAtTime (t, pose_symbol);
     boost::optional<gtsam::Pose3> cloud_pose = mapper_->predictPose (
         pose_symbol);

    /* transfrom point clouds to base_frame */
    Eigen::Affine3d sensor_to_base = Eigen::Affine3d::Identity();
    if (get_sensor_to_base_)
    {
      if(debug_)
      printf ("[ObjectPlugin] Object Plugin: Applying sensor to base transform.\n");
       sensor_to_base = (*get_sensor_to_base_) (t);
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

    // Keep track of indices using indices_base
    std::vector<pcl::PointIndices> indices_base (clusters_base.size ());


    
    gtsam::Values solution;
    if (filter_points_near_planes_)
    {
      solution = mapper_->getSolution ();
    }
    gtsam::Values::ConstFiltered<gtsam::Plane<PointT> > plane_filtered =
        solution.filter<gtsam::Plane<PointT> > ();

    // Save observations made from this pose
    //observations_.insert (std::pair<gtsam::Symbol, CloudPtrVector >(pose_symbol, clusters_base));
    CloudPtrVector filtered_observations;
    std::vector<pcl::PointIndices> filtered_observation_indices;

    // Compute cluster statistics for filtering purposes
    Eigen::Matrix3f clust_cov;
    Eigen::Vector4f clust_centroid = Eigen::Vector4f::Zero ();
    Eigen::Vector4f min_pt;
    Eigen::Vector4f max_pt;
    EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
    EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;

    for (int i = 0; i < clusters_base.size (); i++)
    {
      // Detect nearby planes
      if (filter_points_near_planes_)
      {
        // See what plane (if any) is closest
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
            //closest_sym = key_value.key;
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
          {
            filtered_indices.indices.push_back ( (*indices)[i].indices[j]);

          }
          indices_base[i] = filtered_indices;

          if(debug_)
          printf ("[ObjectPlugin] cluster has %d points, indices has %d points\n",
              clusters_base[i]->points.size (),
              indices_base[i].indices.size ());

        }
      }

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
    std::cout << "[ObjectPlugin] Size of filtered observations: "
        << filtered_observations.size () << std::endl;
    observations_.insert (
        std::pair<gtsam::Symbol, CloudPtrVector> (pose_symbol,
            filtered_observations));

    observation_indices_.insert (
        std::pair<gtsam::Symbol, std::vector<pcl::PointIndices> > (pose_symbol,
            filtered_observation_indices));

    if (cloud_pose && filtered_observations.size () != 0)
    {

      /* Perform temporal segmentation on every new frame and match it to the
       * existing set of objects
       */
      double seg_start = pcl::getTime ();
      CloudPtrVector final_label = segment_propagation_->predictLabels (
          filtered_observations, *cloud_pose, pose_symbol);
      CloudPtrVector matched_cloud = segment_propagation_->final_map_cloud;
      CloudPtrVector final_cloud = segment_propagation_->observations_.at (
          pose_symbol);

      for (int obj_cnt = 0; obj_cnt < final_cloud.size (); obj_cnt++)
      {
        if (final_cloud[obj_cnt]->points.size () == 0)
          continue;

        CloudPtr map_cloud (new Cloud ());
        gtsam::Symbol best_symbol = gtsam::Symbol ('o', obj_cnt);  //plane_filtered.size ());

        if (max_current_size <= obj_cnt)
        {
          // create new object
          Object<PointT> new_object;

          // increase the count
          max_current_size = obj_cnt + 1;

          // fill in the object
          int old_label = segment_propagation_->back_label_.at (obj_cnt);
          new_object.sym = best_symbol;
          new_object.addObservation (pose_symbol, final_cloud[obj_cnt],
              filtered_observation_indices[old_label]);
          new_object.factor_flag.insert (
              std::pair<gtsam::Symbol, int> (pose_symbol, -1));

          map_building_mutex_.lock(); //lock the object map
          object_map.insert (
              std::pair<gtsam::Symbol, Object<PointT> > (best_symbol,
                  new_object));
          map_building_mutex_.unlock(); //unlock the object map
        }
        else
        {

          int old_label = segment_propagation_->back_label_.at (obj_cnt);

          object_map.at (best_symbol).addObservation (pose_symbol,
              final_cloud[obj_cnt], filtered_observation_indices[old_label]);
          object_map.at (best_symbol).factor_flag.insert (
              std::pair<gtsam::Symbol, int> (pose_symbol, -1));
        }

      }
      double seg_end = pcl::getTime ();

      if(verbose_)
      std::cout << "[ObjectPlugin] Segmentation took: " << double (seg_end - seg_start)
          << std::endl;

      //////////////////////////////////////////////////////////////////////
      // Segmentation Ends
      //////////////////////////////////////////////////////////////////////



    }

    /*
          * Find a set of objects from the segments
          * for which the recognition will be called
          */

      /* estimate view pose using base frame pose and sensor_to_base transformation */
       gtsam::Pose3 base_to_sensor(sensor_to_base.inverse().matrix());
      gtsam::Pose3 view_pose= (*cloud_pose)*gtsam::Pose3(sensor_to_base.matrix());

      /* compute camera view direction */
      gtsam::Rot3 view_rotation = view_pose.rotation();
      gtsam::Point3 view_direction_vec = view_rotation.r3();
      double view_direction_norm = view_direction_vec.norm();
      gtsam::Point3 view_direction (view_direction_vec.x ()/view_direction_norm, view_direction_vec.y ()/view_direction_norm,
          view_direction_vec.z ()/view_direction_norm); // camera view direction
      gtsam::Point3 view_center = view_pose.translation ();  //camera center

      if(debug_){
      view_center.print("[ObjectPlugin] view_center");
      view_direction.print("[ObjectPlugin] view_direction");
      }
      CloudPtrVector unoptimized_cloud = segment_propagation_->final_map_cloud;
      for (int obj_cnt = 0; obj_cnt < unoptimized_cloud.size (); obj_cnt++)
      {
        if(debug_)
        std::cout << "[ObjectPlugin] Size of object " << obj_cnt << " is "
            << unoptimized_cloud[obj_cnt]->points.size () << std::endl;
        gtsam::Symbol obj_symbol = gtsam::Symbol ('o', obj_cnt);
        if (unoptimized_cloud[obj_cnt]->points.size () != 0)
        {
          // if the bounding box containing the current frame
          // does not intersect with the given object then train the object
          Eigen::Vector4f obj_centroid;
          pcl::compute3DCentroid (*unoptimized_cloud[obj_cnt], obj_centroid);

          float intersection = computeViewIntersection(view_direction, view_center, obj_centroid);

          std::cout << "[ObjectPlugin] Intersection of object " << obj_cnt << " is: "
              << intersection << std::endl;

          if (intersection == 0 && training_map[obj_symbol] == 0)
          {
            if(debug_)
            std::cout << "[ObjectPlugin] Intersection of object " << obj_cnt
                << " is zero with the cloud" << std::endl;
            training_map[obj_symbol] = 1;

            if(verbose_)
            obj_symbol.print ("[ObjectPlugin] Object symbol pushed: ");
            pushIntoQueue (obj_symbol);
          }
          else if (intersection != 0)
          {
            if (training_map.find (obj_symbol) == training_map.end ())
            {
              training_map.insert (
                  std::pair<gtsam::Symbol, int> (obj_symbol, 0));
            }
            else
            {
              training_map[obj_symbol] = 0;
            }

          }


        }
      }

      //////////////////////////////////////////////////////////////////////
      // Objects estimated
      //////////////////////////////////////////////////////////////////////

      /*
       * Visualize
       */
      double vis_start = pcl::getTime ();

      if (vis_flag_)
      {
        if(verbose_)
        std::cout << "[ObjectPlugin] Visualizing Objects" << std::endl;
        vis_callback_ (object_map, view_center, view_direction);

        //cloud_cv_callback_(pose_symbol, cloud_pose, filtered_observations, t);
      }
      double vis_end = pcl::getTime ();


      if(verbose_)
      std::cout << "[ObjectPlugin] Visualizations took: " << double (vis_end - vis_start)
          << std::endl;




  }

  template<typename PointT>
  void ObjectPlugin<PointT>::update (
      boost::shared_ptr<gtsam::Values>& vis_values,
      boost::shared_ptr<gtsam::NonlinearFactorGraph>& vis_graph)
  {

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
