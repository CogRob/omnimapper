#include <omnimapper/organized_feature_extraction_tbb.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <semantic_scene_understanding/Object.h>
#include <semantic_scene_understanding/ObjectList.h>
#include <pointing_object_evaluation/ObjectInfo.h>
#include <pointing_object_evaluation/ObjectInfos.h>
#include <tf/transform_listener.h>

template <typename PointT>
class OrganizedSegmenationNode
{
  typedef boost::posix_time::ptime Time;
  typedef pcl::PointCloud<PointT> Cloud;
  typedef typename Cloud::Ptr CloudPtr;
  typedef typename Cloud::ConstPtr CloudConstPtr;
  typedef typename pcl::PointCloud<pcl::Label> LabelCloud;
  typedef typename LabelCloud::Ptr LabelCloudPtr;
  typedef typename LabelCloud::ConstPtr LabelCloudConstPtr;

  protected:
    ros::NodeHandle n_;
    ros::Subscriber pointcloud_sub_;
    ros::Publisher marker_array_pub_;
    ros::Publisher object_list_pub_;

    omnimapper::OrganizedFeatureExtractionTBB<PointT> ofe_;

    boost::thread ofe_spin_;
  tf::TransformListener tf_listener_;


  public:
    OrganizedSegmenationNode () 
      : n_ ("~"),
	tf_listener_()
    {
      // Subscribe to clouds
      pointcloud_sub_ = n_.subscribe ("/camera/depth_registered/points", 1, &OrganizedSegmenationNode<PointT>::cloudCallback, this);
      
      // Publish Visualization Markers
      marker_array_pub_ = n_.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 0);

      // Publish Object List
      object_list_pub_ = n_.advertise<pointing_object_evaluation::ObjectInfos>("/object_infos", 0);

      // Hook up object callback
      typename boost::function<
        void (std::vector<CloudPtr>, Time t,
              boost::optional<std::vector<pcl::PointIndices> >)> object_cluster_callback =
      boost::bind (&OrganizedSegmenationNode<PointT>::clusterCloudCallback,
                   this, _1, _2, _3);
      ofe_.setClusterCloudCallback (object_cluster_callback);
      
      // Start it spinning
      ofe_.spin ();
    }

    void
    cloudCallback (const sensor_msgs::PointCloud2ConstPtr& msg)
    {
      //ROS_INFO ("SSU Node: Got a cloud.");
      CloudPtr cloud (new Cloud ());
      pcl::fromROSMsg <PointT>(*msg, *cloud);
      ofe_.cloudCallback (cloud);
      //ofe_.spinOnce ();
    }

    void clusterCloudCallback (std::vector<CloudPtr> clusters,
                               Time t, 
                               boost::optional<std::vector<pcl::PointIndices> >)
    {
      //ROS_INFO ("SSU Node: Got %d clusters!", clusters.size ());
      // Publish these to vis and object topics
      visualization_msgs::MarkerArray object_centroid_markers;
      std::vector<Eigen::Vector4f> centroids;

      ros::Time curr_time = ros::Time::now();

      pointing_object_evaluation::ObjectInfos obj_list;
      obj_list.header.frame_id = std::string("/camera_rgb_optical_frame");
      obj_list.header.stamp = curr_time;

      for (int i = 0; i < clusters.size (); i++)
	{
	  Eigen::Vector4f centroid;
	  pcl::compute3DCentroid ((*clusters[i]), centroid);
	  if (centroid.norm() < 1.7)
	    {
	      /*
	      // Convert it to the other frame
	      geometry_msgs::PointStamped p_in, p_out;
	      p_in.point.x = centroid[0];
	      p_in.point.y = centroid[1];
	      p_in.point.z = centroid[2];
	      p_in.header.stamp = curr_time;
	      p_in.header.frame_id = std::string("/camera_rgb_optical_frame");
	      
	      try{
		tf_listener_.waitForTransform("/camera_depth_frame", "/camera_rgb_optical_frame", curr_time, ros::Duration(0.2));
	      }
	      catch(tf::TransformException ex) {
		ROS_INFO("TF Sucks: %s", ex.what());
		return;
	      }
	      tf_listener_.transformPoint(std::string("/camera_depth_frame"), p_in, p_out);
	      */ 
	      
	      centroids.push_back (centroid);
	      
	      pointing_object_evaluation::ObjectInfo obj;
	      sensor_msgs::PointCloud2 obj_cloud_msg;
	      obj.header.stamp = curr_time;
	      pcl::toROSMsg ((*clusters[i]), obj.cloud);
	      obj.centroid.x = centroid[0];//p_out.point.x;
	      obj.centroid.y = centroid[1];//p_out.point.y;
	      obj.centroid.z = centroid[2];//p_out.point.z;
	      obj_list.objects.push_back (obj);
	      
	      visualization_msgs::Marker object_centroid_marker;
	      object_centroid_marker.header.frame_id = "/camera_rgb_optical_frame";//"/camera_depth_frame";
	      object_centroid_marker.header.stamp = ros::Time::now ();
	      object_centroid_marker.type = visualization_msgs::Marker::SPHERE;
	      object_centroid_marker.action = visualization_msgs::Marker::ADD;
	      object_centroid_marker.ns = "object_centroids";
	      object_centroid_marker.id = i;
	      object_centroid_marker.lifetime = ros::Duration(1.0);
	      object_centroid_marker.color.r = 0.2f;
	      object_centroid_marker.color.g = 0.2f;
	      object_centroid_marker.color.b = 1.0f;
	      object_centroid_marker.color.a = 0.5;
	      object_centroid_marker.pose.position.x = centroid[0];//p_out.point.x;//centroid[0];
	      object_centroid_marker.pose.position.y = centroid[1];//p_out.point.y;//centroid[1];
	      object_centroid_marker.pose.position.z = centroid[2];//p_out.point.z;//centroid[2];
	      object_centroid_marker.pose.orientation.x = 0;
	      object_centroid_marker.pose.orientation.y = 0;
	      object_centroid_marker.pose.orientation.z = 0;
	      object_centroid_marker.pose.orientation.w = 1;
	      object_centroid_marker.scale.x = 0.06;
	      object_centroid_marker.scale.y = 0.06;
	      object_centroid_marker.scale.z = 0.06;
	      object_centroid_markers.markers.push_back (object_centroid_marker);
	    }
	}
      
      marker_array_pub_.publish (object_centroid_markers);

      // Publish these to object messages
      object_list_pub_.publish (obj_list);
      
    }

};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "OrganizedSegmenationNode");
  OrganizedSegmenationNode<pcl::PointXYZRGBA> ssu_node;
  ros::spin ();
}
