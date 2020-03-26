#include <omnimapper_ros/omnimapper_ros.h>
#include <pcl/common/time.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/filesystem.hpp>

typedef pcl::PointXYZRGBA PointT;
typedef typename pcl::PointCloud<PointT> Cloud;
typedef typename Cloud::Ptr CloudPtr;
typedef typename Cloud::ConstPtr CloudConstPtr;

template <typename PointT>
class OmniMapperROSOpenNI {
 public:
  OmniMapperROSOpenNI()
      : nh_("~"), omnimapper_(nh_), ni_grabber_("#1"), publish_cloud_(true) {
    pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/grabber_cloud", 0);

    boost::function<void(const CloudConstPtr&)> f =
        boost::bind(&OmniMapperROSOpenNI<PointT>::cloudCallback, this, _1);
    boost::signals2::connection c = ni_grabber_.registerCallback(f);
    ni_grabber_.start();

    ROS_INFO("mapper openni -- constructed.\n");
  }

  void cloudCallback(const CloudConstPtr& cloud) {
    ROS_INFO("mapper openni -- cloud callback\n");
    ros::Time now = ros::Time::now();

    // Convert to ROS MSG
    sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*cloud, *cloud_msg);
    cloud_msg->header.stamp = now;
    cloud_msg->header.frame_id = "/current_pose";

    sensor_msgs::PointCloud2ConstPtr const_cloud_msg(cloud_msg);
    // Send to mapper
    omnimapper_.CloudCallback(const_cloud_msg);

    // Publish cloud
    if (publish_cloud_) {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(
          new pcl::PointCloud<pcl::PointXYZRGB>());
      copyPointCloud(*cloud, *rgb_cloud);
      sensor_msgs::PointCloud2Ptr rgb_cloud_msg(new sensor_msgs::PointCloud2());
      pcl::toROSMsg(*rgb_cloud, *rgb_cloud_msg);
      rgb_cloud_msg->header.stamp = now;
      rgb_cloud_msg->header.frame_id = "/current_pose";

      pc_pub_.publish(rgb_cloud_msg);
    }
  }

 protected:
  ros::NodeHandle nh_;
  OmniMapperROS<PointT> omnimapper_;
  pcl::OpenNIGrabber ni_grabber_;
  ros::Publisher pc_pub_;
  bool publish_cloud_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "OmniMapperROSOpenNI");

  printf("Starting OmniMapperROS Openni...\n");

  OmniMapperROSOpenNI<pcl::PointXYZRGBA> mapper_openni;

  printf("constructor complete\n");

  // spinner.start ();

  // PCL Openni Grabber, since ROS node seems to be unreliable
  // pcl::OpenNIGrabber ni_grabber ("#1");
  // boost::function<void (const CloudConstPtr&)> f = boost::bind
  // (&OmniMapperROS<pcl::PointXYZRGBA>::cloudCallback, &omnimapper, _1);
  // boost::signals2::connection c = ni_grabber.registerCallback (f);
  // ni_grabber.start ();

  ros::spin();
}
