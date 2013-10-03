
#include <omnimapper/object_discovery.h>

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

template<typename PointT>
ObjectDiscovery<PointT>::ObjectDiscovery(): max_current_size(0),
max_object_size(0), object_dir_("/home/siddharth/kinect/"){

}


template<typename PointT>
float ObjectDiscovery<PointT>::computeJaccardIndex(Eigen::Vector4f min_pt_1,
		Eigen::Vector4f min_pt_2, Eigen::Vector4f max_pt_1,
		Eigen::Vector4f max_pt_2) {

	// compute intersection of boxes
	float minX = min_pt_1[0];
	float minY = min_pt_1[1];
	float minZ = min_pt_1[2];
	float maxX = max_pt_1[0];
	float maxY = max_pt_1[1];
	float maxZ = max_pt_1[2];

	float minX1 = min_pt_2[0];
	float minY1 = min_pt_2[1];
	float minZ1 = min_pt_2[2];
	float maxX1 = max_pt_2[0];
	float maxY1 = max_pt_2[1];
	float maxZ1 = max_pt_2[2];

	float inter_area = MAX(MIN(maxX,maxX1)-MAX(minX,minX1),0)
			* MAX(MIN(maxY,maxY1)-MAX(minY,minY1),0)
			* MAX(MIN(maxZ,maxZ1)-MAX(minZ,minZ1),0);

	float areaA = (maxX - minX) * (maxY - minY) * (maxZ - minZ);
	float areaB = (maxX1 - minX1) * (maxY1 - minY1) * (maxZ1 - minZ1);

	float jaccard_index = inter_area / (areaA + areaB - inter_area);

//    std::cout << "[inside jaccard] " << inter_area << " " << areaA << " "
//        << areaB << std::endl;
	return jaccard_index;
}

template<typename PointT>
void ObjectDiscovery<PointT>::loadRepresentations(std::string object_location) {

  object_dir_ = object_location;
	std::cout << "Inside loadDesc" << std::endl;
	pcl::SIFTKeypoint<PointT, pcl::PointXYZI>* sift3D = new pcl::SIFTKeypoint<
			PointT, pcl::PointXYZI>;
	sift3D->setScales(0.01f, 3, 2);
	sift3D->setMinimumContrast(0.0);

	boost::shared_ptr<pcl::Keypoint<PointT, pcl::PointXYZI> > keypoint_detector;
	keypoint_detector.reset(sift3D);

	/* and descriptors */

	pcl::SHOTColorEstimationOMP<PointT, pcl::Normal, pcl::SHOT1344>* shot =
			new pcl::SHOTColorEstimationOMP<PointT, pcl::Normal, pcl::SHOT1344>;
	shot->setRadiusSearch(0.1);
	typename pcl::Feature<PointT, pcl::SHOT1344>::Ptr feature_extractor(shot);
	pcl::PointCloud<pcl::SHOT1344>::Ptr features(
			new pcl::PointCloud<pcl::SHOT1344>);

	/* initialize correspondence estimator */
	correspondence_estimator.reset(
			new ObjectRecognition<pcl::SHOT1344>(keypoint_detector,
					feature_extractor));

	std::cout << "Descriptor Loaded " << std::endl;

	/* load object descriptors */

	int max_segment = correspondence_estimator->loadDatabase(object_location);

	max_object_size = max_segment + 1;
	max_current_size = max_object_size;
	std::cout << "Size of max_segment " << max_segment + 1 << std::endl;

}

template<typename PointT>
int ObjectDiscovery<PointT>::findMin(int segment, std::vector<int> seg_vec) {
	int min_obj = segment;

	for (int i = 0; i < seg_vec.size(); i++) {
		if (min_obj > seg_vec[i])
			min_obj = seg_vec[i];
	}

	return min_obj;
}

template<typename PointT>
int ObjectDiscovery<PointT>::findLabel(int segment) {
	int label = segment;
	while (segment_arr[label] != label) {
		if (segment_arr[label] == -1) {
			// no label is assigned yet
			segment_arr[label] = label;
			break;
		}
		label = segment_arr[label];
	}
	return label;

}


template<typename PointT>
void ObjectDiscovery<PointT>::reconstructSurface(typename pcl::PointCloud<PointT>::Ptr merged, int id) {
	std::cout << "surface reconstruction..." << std::flush;

	// apply grid filtering to reduce amount of points as well as to make them uniform distributed
	pcl::VoxelGrid<pcl::PointXYZRGBA> voxel_grid;
	voxel_grid.setInputCloud(merged);
	voxel_grid.setLeafSize(0.002f, 0.002f, 0.002f);
	voxel_grid.setDownsampleAllData(true);
	voxel_grid.filter(*merged);

	std::cout << "ran voxel grid" << std::endl;

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr vertices(
			new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::copyPointCloud(*merged, *vertices);

	std::cout << "copied" << std::endl;

	pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::PointXYZRGBNormal> normal_estimation;
	normal_estimation.setSearchMethod(
			pcl::search::Search<pcl::PointXYZRGBA>::Ptr(
					new pcl::search::KdTree<pcl::PointXYZRGBA>));
	normal_estimation.setRadiusSearch(0.01);
	normal_estimation.setInputCloud(merged);
	normal_estimation.compute(*vertices);
	std::cout << "normal estimated" << std::endl;


	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree(
			new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
	tree->setInputCloud(vertices);

	std::cout << "set cloud" << std::endl;

	pcl::PolygonMesh surface_;


	/*
	  pcl::PointCloud<pcl::PointXYZRGBNormal> mls_points;
	  pcl::MovingLeastSquares<pcl::PointXYZRGBA, pcl::PointXYZRGBNormal> mls;
	  mls.setComputeNormals (true);
	  mls.setInputCloud (*merged);
	  mls.setPolynomialFit (true);
	  mls.setSearchMethod (tree);
	  mls.setSearchRadius (0.03);
	  mls.process (mls_points);
		std::string pcd_file = "/home/siddharth/kinect/tsdf_models/" + boost::lexical_cast<std::string>(id) +".pcd";
		pcl::io::savePCDFile (pcd_file, mls_points);
*/


  boost::shared_ptr<pcl::PCLSurfaceBase<pcl::PointXYZRGBNormal> > surface_reconstruction;
/*
  pcl::MarchingCubes<pcl::PointXYZRGBNormal>* mc = new pcl::MarchingCubesHoppe<pcl::PointXYZRGBNormal>;
  mc->setIsoLevel (0.001f);
  mc->setGridResolution (1024, 1024, 1024);
  surface_reconstruction.reset(mc);
*/

  pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal>* gp3 = new pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal>;
    gp3->setSearchRadius (1.25);
    gp3->setMu (10.5);
    gp3->setMaximumNearestNeighbors (100);
    gp3->setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3->setMinimumAngle(M_PI/18); // 10 degrees
    gp3->setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3->setNormalConsistency(true);
    surface_reconstruction.reset(gp3);


	surface_reconstruction->setSearchMethod(tree);
	surface_reconstruction->setInputCloud(vertices);
	surface_reconstruction->reconstruct(surface_);

	std::cout << "reconstruction" << std::endl;

	std::cout << "OK" << std::endl;

	std::string output_file = object_dir_ + "tsdf_models/" + boost::lexical_cast<std::string>(id) +".ply";
	pcl::io::savePLYFileBinary (output_file, surface_);

}

#if 0
template<typename PointT>
void ObjectDiscovery<PointT>::generateObjectModel(gtsam::Object<PointT> object, Eigen::Vector4f obj_centroid, int id)
{
printf ("starting generateTSDF\n");
// Make a TSDF

//tsdf->setGridSize (10., 10., 10.); // 10m x 10m x 10m
//tsdf->setGridSize (30.0, 30.0, 30.0);
tsdf->setGridSize (10.0, 10.0, 10.0);
tsdf->setResolution (2048, 2048, 2048);
//tsdf->setResolution (2048, 2048, 2048); // Smallest sell cize = 10m / 2048 = about half a centimeter
//tsdf->setResolution (4096, 4096, 4096);


Eigen::Affine3d object_center;
object_center.translation() = Eigen::Vector3d(obj_centroid[0], obj_centroid[1],
			obj_centroid[2]);
Eigen::Affine3d tsdf_center = Eigen::Affine3d::Identity (); // Optionally offset the center
tsdf->setGlobalTransform (object_center);
//tsdf->setDepthTruncationLimits ();
//tsdf->setDepthTruncationLimits (0.3, 10.0);
//tsdf->setWeightTruncationLimit (100.0);
tsdf->reset (); // Initialize it to be empty

std::map<gtsam::Symbol, CloudPtr> cluster = object.clusters_;
std::map<gtsam::Symbol, pcl::PointIndices> indices = object.indices_;

typename std::map<gtsam::Symbol, CloudPtr>::iterator it;

	Cloud transformed_cloud_opt_;


	for (it = cluster.begin(); it != cluster.end(); it++) {

		gtsam::Symbol sym = it->first;
		CloudPtr cloud = it->second;


		boost::optional<gtsam::Pose3> cloud_pose = mapper_->predictPose(sym);

		if (cloud_pose) {
			CloudPtr map_cloud(new Cloud());
			map_cloud->width = 640;
			map_cloud->height = 480;
			map_cloud->resize(map_cloud->width*map_cloud->height);

			for(int i=0; i< map_cloud->width*map_cloud->height; i++){
			//	std::cout << i << std::endl;
				PointT invalid_pt;
				invalid_pt.x = std::numeric_limits<float>::quiet_NaN();
				invalid_pt.y = std::numeric_limits<float>::quiet_NaN();
				invalid_pt.z = std::numeric_limits<float>::quiet_NaN();
				invalid_pt.r = 1; invalid_pt.g = 0; invalid_pt.b = 0; invalid_pt.a=1;
				map_cloud->points[i] = invalid_pt;

			}

			pcl::PointIndices clust_indices = indices.at(sym);
			std::cout << "Inside the loop: " << cloud->points.size() << " Size of clust indices " << clust_indices.indices.size() << std::endl;
		//	std::cout << "Size of clust indices " << clust_indices.indices.size() << std::endl;

			for(int i=0; i< clust_indices.indices.size(); i++){
				std::cout << "indices: " << clust_indices.indices[i] << std::endl;
				map_cloud->points[clust_indices.indices[i]] = cloud->points[i];
			}


			gtsam::Pose3 sam_pose = *cloud_pose;
			//const gtsam::Rot3 rot;
		//	const gtsam::Point3 centroid_pt(obj_centroid[0], obj_centroid[1], obj_centroid[2]);
			//gtsam::Pose3 centroid_tform(rot, centroid_pt);
		//	gtsam::Pose3 inv_tform = centroid_tform.inverse();
			//sam_pose = inv_tform*sam_pose; // order of multiplication should be kept in mind

			Eigen::Matrix4f map_tform = sam_pose.matrix().cast<float>();
			Eigen::Affine3d tform;
			gtsam::Quaternion sam_quat = sam_pose.rotation().toQuaternion();
			tform = Eigen::Quaterniond(sam_quat.w(), sam_quat.x(), sam_quat.y(),
					sam_quat.z());
			tform.translation() = Eigen::Vector3d(sam_pose.x(), sam_pose.y(),
				sam_pose.z());

			/* testing the transformation */
			CloudPtr vis_cloud(new Cloud());
			pcl::transformPointCloud(*map_cloud, *vis_cloud, tform);
			transformed_cloud_opt_+= *vis_cloud;


			//Eigen::Affine3d tform_inv = tform.inverse();
			//pcl::transformPointCloud (*frame_cloud, *map_cloud, map_tform);
			//Eigen::Affine3d tform (Eigen::Quaterniond(sam_quat[0],sam_quat[1],sam_quat[2],sam_quat[3]), Eigen::Vector3d (sam_pose.x (), sam_pose.y (), sam_pose.z ()));

			pcl::PointCloud<pcl::Normal> empty_normals;
			pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
			ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
			ne.setMaxDepthChangeFactor(0.02f);
			ne.setNormalSmoothingSize(20.0f);
			ne.setDepthDependentSmoothing(true);
			//ne.setRadiusSearch (0.1);
			//ne.setInputCloud (frame_cloud);
			//ne.compute (empty_normals);
			//empty_normals.resize (frame_cloud->points.size ());

			printf("Cloud has: %d normals has: %d\n",
					cloud->points.size(), empty_normals.points.size());

			if (cloud->points.size() > 0){
				std::cout << "Cloud integrated " << std::endl;
				bool integration_flag = tsdf->integrateCloud<pcl::PointXYZRGBA, pcl::Normal>(*map_cloud,
						empty_normals, tform); // Integrate the cloud
				std::cout << "Integration done: " << integration_flag << std::endl;
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

	std::string vis_file = "/home/siddharth/kinect/test_models/" + boost::lexical_cast<std::string>(id) +".pcd";
				pcl::io::savePCDFileASCII (vis_file, transformed_cloud_opt_);


std::string vol_file = "/home/siddharth/kinect/tsdf_models/" + boost::lexical_cast<std::string>(id) +".vol";
tsdf->save (vol_file); // Save it?

// Maching Cubes
cpu_tsdf::MarchingCubesTSDFOctree mc;
mc.setInputTSDF (tsdf);
mc.setColorByConfidence (true);
mc.setColorByRGB (false);
//mc.setMinWeight (0.1);
pcl::PolygonMesh mesh;
mc.reconstruct (mesh);

std::string output_file = "/home/siddharth/kinect/tsdf_models/" + boost::lexical_cast<std::string>(id) +".ply";
pcl::io::savePLYFileBinary (output_file, mesh);

// Render from xo
Eigen::Affine3d init_pose = Eigen::Affine3d::Identity ();
pcl::PointCloud<pcl::PointNormal>::Ptr raytraced = tsdf->renderView (init_pose);
std::string render_file = "/home/siddharth/kinect/tsdf_models/rendered_" + boost::lexical_cast<std::string>(id) +".pcd";

pcl::io::savePCDFileBinary (render_file, *raytraced);

}

#endif
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


template<typename PointT>
void ObjectDiscovery<PointT>::mergeClouds() {
	std::map<int, std::vector<int> >::iterator it;


	/* clear previous content */
	std::string to_remove = object_dir_ + "/merged_object_models/";
	boost::filesystem::path path_to_remove(to_remove);
	for (boost::filesystem::directory_iterator end_dir_it, it(path_to_remove);
			it != end_dir_it; ++it) {
		remove_all(it->path());
	}


	std::string to_remove_1 = object_dir_ + "/merged_object_templates/";
	boost::filesystem::path path_to_remove_1(to_remove_1);
	for (boost::filesystem::directory_iterator end_dir_it, it(path_to_remove_1);
			it != end_dir_it; ++it) {
		remove_all(it->path());
	}

	std::string to_remove_2 = object_dir_ + "/merged_pose_templates/";
	boost::filesystem::path path_to_remove_2(to_remove_2);
	for (boost::filesystem::directory_iterator end_dir_it, it(path_to_remove_2);
			it != end_dir_it; ++it) {
		remove_all(it->path());
	}



	std::cout << "Size of max object " << max_object_size - 1 << std::endl;
	segment_arr.resize(max_object_size, -1);

	std::cout << "MATCH GRAPH: " << std::endl;
	for (it = match_graph.begin(); it != match_graph.end(); it++) {
		int obj_id = it->first;
		std::cout << "object id " << obj_id << " ";
		std::vector<int> matching_objects = it->second;

		int min_obj = findMin(obj_id, matching_objects);
		int label = findLabel(obj_id);
		if(label>min_obj)label=min_obj;
		for (int i = 0; i < matching_objects.size(); i++) {
			segment_arr[matching_objects[i]] = label;
			std::cout << matching_objects[i] << " ";
		}
		segment_arr[obj_id]=label;
		std::cout << "Minimum: " << min_obj << " Label: " << label << std::endl;
		std::cout << std::endl;
	}

	std::map<int, std::vector<int> > object_map;
	for (int i = 0; i < segment_arr.size(); i++) {
		object_map.insert(
				std::pair<int, std::vector<int> >(i, std::vector<int>()));
		if (segment_arr[i] == -1)
			continue;
			std::cout << i << " " << segment_arr[i] << std::endl;
		object_map.at(segment_arr[i]).push_back(i);
	}
	std::cout << std::endl << "---------------" << std::endl;
	std::cout << "OBJECT GRAPH: " << std::endl;

	correspondence_estimator->segment_object.clear();
	for (it = object_map.begin(); it != object_map.end(); it++) {
		int obj_id = it->first;
		std::vector<int> matching_objects = it->second;
		if (matching_objects.size() == 0)
			continue;

		std::cout << "object id " << obj_id << " ";
		pcl::PointCloud<pcl::PointXYZRGBA> merged_cloud;
		std::vector<gtsam::Pose3> new_pose_array;
		for (int i = 0; i < matching_objects.size(); i++) {
			std::cout << matching_objects[i] << " ";
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr temp_cloud = map_cloud.at(
					matching_objects[i]);
			merged_cloud += *temp_cloud;

			std::vector<gtsam::Pose3> pose_array =
					correspondence_estimator->pose_map.at(matching_objects[i]).pose_vector;
			new_pose_array.insert(new_pose_array.end(), pose_array.begin(),
					pose_array.end());

		}

		if (correspondence_estimator->segment_object.find(obj_id)
				== correspondence_estimator->segment_object.end()) {
			// segment not found in the database
			std::map<int, int> temp_object;
			temp_object[obj_id] = 1;
			correspondence_estimator->segment_object[obj_id] = temp_object;
		}

		// save merged clouds
		std::string output_file = object_dir_ + "/merged_object_models/"
				+ boost::lexical_cast<std::string>(obj_id) + ".pcd";
		pcl::io::savePCDFileASCII(output_file, merged_cloud);

		// save new poses
		PoseVector pose_vec;
		pose_vec.pose_vector = new_pose_array;
		std::string pose_filename = object_dir_ + "/merged_pose_templates/object_"
				 + boost::lexical_cast<std::string>(obj_id);
		correspondence_estimator->savePoseArray(pose_filename, pose_vec);
		correspondence_estimator->pose_map.at(obj_id) = pose_vec;

		std::string output_filename = object_dir_ + "/merged_object_templates/object_"
				 + boost::lexical_cast<std::string>(obj_id) + "_0";
		correspondence_estimator->computeAndStoreDescriptor(
				merged_cloud.makeShared(), output_filename);



		//
		std::cout << std::endl;

	}

	std::string merged_mapping_file = object_dir_ + "/merged_mapping.txt";
	std::string object_stats_file = object_dir_ + "/merged_stats.txt";
	correspondence_estimator->saveMapping(merged_mapping_file);
	correspondence_estimator->saveObjectStats(object_stats_file);

	std::cout << std::endl << "---------------" << std::endl;
	std::cout << "OVERLAP GRAPH" << std::endl;
	for (it = graph.begin(); it != graph.end(); it++) {
		int obj_id = it->first;
		std::cout << "object id " << obj_id << " ";
		std::vector<int> matching_objects = it->second;

		for (int i = 0; i < matching_objects.size(); i++) {
			std::cout << matching_objects[i] << " ";
		}
		std::cout << std::endl;
	}

	std::cout << std::endl << "---------------" << std::endl;
}

template<typename PointT>
void ObjectDiscovery<PointT>::createGraph() {

	std::map<int, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>::iterator it;
	std::map<int, std::pair<Eigen::Vector4f, Eigen::Vector4f> > min_max;
	std::map<int, std::pair<Eigen::Vector4f, Eigen::Vector4f> >::iterator min_max_iterator_1;
	std::map<int, std::pair<Eigen::Vector4f, Eigen::Vector4f> >::iterator min_max_iterator_2;

	int max_size =-1;
	for (it = map_cloud.begin(); it != map_cloud.end(); it++) {

		// compute bounding box and find other clouds having intersection

		int obj_id = it->first;
		Eigen::Vector4f min_pt;
		Eigen::Vector4f max_pt;

		pcl::PointCloud<PointT> cloud = *(it->second);
		getMinMax3D(cloud, min_pt, max_pt);

		std::pair<Eigen::Vector4f, Eigen::Vector4f> min_max_pair =
				std::make_pair(min_pt, max_pt);
		min_max.insert(
				std::pair<int, std::pair<Eigen::Vector4f, Eigen::Vector4f> >(
						obj_id, min_max_pair));

		if(obj_id > max_size)
			max_size = obj_id;

	}

	max_size++;
	max_object_size = max_size;
	max_current_size = max_object_size;


	for (min_max_iterator_1 = min_max.begin();
			min_max_iterator_1 != min_max.end(); min_max_iterator_1++) {

		int obj_id = min_max_iterator_1->first;
		std::cout << "object: " << obj_id << " ";
		std::vector<int> nearby_segments;
		graph.insert(
				std::pair<int, std::vector<int> >(obj_id, nearby_segments));
		match_graph.insert(
				std::pair<int, std::vector<int> >(obj_id, nearby_segments));

		for (min_max_iterator_2 = min_max.begin();
				min_max_iterator_2 != min_max.end(); min_max_iterator_2++) {

			int obj_id_2 = min_max_iterator_2->first;
			if (obj_id_2 == obj_id)
				continue;

			float jaccard_index = computeJaccardIndex(
					min_max_iterator_1->second.first,
					min_max_iterator_2->second.first,
					min_max_iterator_1->second.second,
					min_max_iterator_2->second.second);

			if (jaccard_index != 0) {
				graph.at(obj_id).push_back(obj_id_2);

				int result = correspondence_estimator->matchToFile(
						map_cloud.at(obj_id), map_cloud.at(obj_id_2));

				if (result >= 12) {
					match_graph.at(obj_id).push_back(obj_id_2);
				}
				std::cout << obj_id_2 << " ";
			}

		}
		std::cout << std::endl;

	}

}

template<typename PointT>
void ObjectDiscovery<PointT>::createFinalCloud(std::string dir) {

	boost::filesystem::directory_iterator end_itr;

	for (boost::filesystem::directory_iterator itr(dir); itr != end_itr;
			++itr) {
		std::string file_name = itr->path().string();
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(
				new pcl::PointCloud<pcl::PointXYZRGBA>);

		if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(file_name, *cloud) == -1) //* load the file
				{
			PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		}

		std::vector<std::string> parsed_str, object_name;
		boost::split(parsed_str, itr->path().string(), boost::is_any_of("/"));
		boost::split(object_name, parsed_str[parsed_str.size() - 1],
				boost::is_any_of("."));

		int obj_id_val = boost::lexical_cast<int>(object_name[0]);
//            std::cout << obj_id_val << " " << std::endl;
		map_cloud.insert(
				std::pair<int, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>(
						obj_id_val, cloud));

	}

}

int main() {

	std::string dir = "/home/siddharth/kinect/object_models";
	ObjectDiscovery<pcl::PointXYZRGBA> process_cloud;
	process_cloud.createFinalCloud(dir);
/*
	std::map<int, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>::iterator it;
	for(it = process_cloud.map_cloud.begin(); it!= process_cloud.map_cloud.end(); it++){

		process_cloud.reconstructSurface(it->second, it->first);
	}*/

	process_cloud.createGraph();
	process_cloud.mergeClouds();
	return 0;

}

template class ObjectDiscovery<pcl::PointXYZRGBA> ;

