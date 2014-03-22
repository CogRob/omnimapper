#include <omnimapper_ros/omnimapper_ros.h>

int main (int argc, char** argv)
{
  ros::init (argc, argv, "OmniMapperROSNode");
  //ProfilerStart ("omnimapper_ros_node.prof");
  ros::NodeHandle nh;
  OmniMapperROS<PointT> omnimapper (nh);

  // Start ROS Spinning
  ros::AsyncSpinner spinner (4);
  spinner.start ();
  
  // Create list of fr1 datasets
  std::vector<std::string> datasets;
  std::string threesixty_name ("rgbd_dataset_freiburg1_360");
  datasets.push_back(threesixty_name);
  std::string desk2_name("rgbd_dataset_freiburg1_desk2");
  datasets.push_back(desk2_name);
  std::string floor_name("rgbd_dataset_freiburg1_floor");
  datasets.push_back(floor_name);
  std::string plant_name("rgbd_dataset_freiburg1_plant");
  datasets.push_back(plant_name);
  std::string teddy_name("rgbd_dataset_freiburg1_teddy");
  datasets.push_back(teddy_name);
  std::string desk1_name ("rgbd_dataset_freiburg1_desk");
  datasets.push_back (desk1_name);
  std::string xyz_name ("rgbd_dataset_freiburg1_xyz");
  datasets.push_back (xyz_name);
  std::string rpy_name ("rgbd_dataset_freiburg1_rpy");
  datasets.push_back(rpy_name);

  // ICP Leaf Size
  double leaf_size_min = 0.01; // a centimeter
  double leaf_size_max = 0.10;
  double leaf_size_step = 0.01; // a centimeter
  
  // ICP Correspondence Distance
  double correspondence_dist_min = 0.05;// 1cm
  double correspondence_dist_max = 0.2;// 50cm
  double correspondence_dist_step = 0.01;// half a centimeter

  // ICP Score Threshold
  double score_thresh_min = 0.01;
  double score_thresh_max = 0.1;
  double score_thresh_step = 0.01;

  // For each dataset
  for (int i = 0; i < datasets.size (); i++)
  {
    std::string base_path ("/home/atrevor/data/tum_rgbd/");
    base_path.append (datasets[i]);
    // Set up dataset
    std::string associated_filename = base_path + std::string ("/associated.txt");
    std::string ground_truth_filename = base_path + std::string ("/groundtruth.txt");
    std::string pcd_path = base_path + std::string ("/pcd/rgb");
    
    std::cout << "Processing dataset at: " << base_path << std::endl;

    // For each leaf size
    for (double leaf_size = leaf_size_min; leaf_size <= leaf_size_max; leaf_size += leaf_size_step)
    {
      ros::param::set ("/benchmark_node/icp_leaf_size", leaf_size);
      
      //for (double correspondence_dist = correspondence_dist_min; correspondence_dist <= correspondence_dist_max; correspondence_dist += correspondence_dist_step)
      //{
      double correspondence_dist = 0.1;
      ros::param::set("/benchmark_node/icp_correspondence_distance", correspondence_dist);
      
      // Create an output filename
      char output_name[4096];
      sprintf (output_name, "trajectory_%s_leafsize_%lf_corrdist_%lf.txt", datasets[i].c_str (), leaf_size, correspondence_dist);
      std::string output_trajectory_filename ("/home/atrevor/benchmark/icp_seq/trajectories/");
      output_trajectory_filename.append ((std::string (output_name)));
      std::string output_timing_filename ("/home/atrevor/benchmark/icp_seq/timing/");
      output_timing_filename.append ((std::string (output_name)));
      
      std::cout << "Associated: " << associated_filename << std::endl;
      std::cout << "Ground Truth: " << ground_truth_filename << std::endl;
      std::cout << "PCD Path: " << pcd_path << std::endl;
      std::cout << "Output Trajectory filename: " << output_trajectory_filename << std::endl;
      std::cout << "Output Timing filename: " << output_timing_filename << std::endl;
      omnimapper.runEvaluation (associated_filename, ground_truth_filename, pcd_path, output_trajectory_filename, output_timing_filename);
      
      //}
      
    }
    

  }


  // Create output filename
  //char output[4096];
  //sprintf ("%s/output/icp_seq_leaf_size_%lf_inf", data_root, leaf_size

  // std::string associated_filename ("/home/atrevor/data/tum_rgbd/rgbd_dataset_freiburg1_desk/associated.txt");
  // std::string ground_truth_filename ("/home/atrevor/data/tum_rgbd/rgbd_dataset_freiburg1_desk/groundtruth.txt");
  // std::string pcd_path ("/home/atrevor/data/tum_rgbd/rgbd_dataset_freiburg1_desk/pcd/rgb");
  // std::string output_filename ("/home/atrevor/data/tum_rgbd/rgbd_dataset_freiburg1_desk/output.txt");
  // std::string output2_filename ("/home/atrevor/data/tum_rgbd/rgbd_dataset_freiburg1_desk/output2.txt");

  // omnimapper.runEvaluation (associated_filename, ground_truth_filename, pcd_path, output_filename);
  // omnimapper.runEvaluation (associated_filename, ground_truth_filename, pcd_path, output2_filename);

  printf ("Done!\n");

  return (0);
//ros::spin ();
  //ProfilerStop ();
}
