/*
 * Software License Agreement (BSD License)
 *
 *  OmniMapper
 *  Copyright (c) 2012-, Georgia Tech Research Corporation,
 *  Atlanta, Georgia 30332-0415
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <organized_segmentation_tools/organized_segmentation_tbb.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/console/parse.h>

/** 
 * \brief Demonstration of OrganizedFeatureExtractionTBB, based on
 * OrganizedConnectedComponentSegmentation, which allows connected
 * components to be found within organized point cloud data, given a
 * comparison function. Used in this instance for plane segmentation
 * and euclidean clustering.
 *
 * \note If you use this code in any academic work, please cite
 *       the appropriate references, which can be found in the
 *       header file.
 *
 * \author Alex Trevor
 */

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef typename Cloud::Ptr CloudPtr;
typedef typename Cloud::ConstPtr CloudConstPtr;
typedef pcl::PointCloud<pcl::Label> LabelCloud;
typedef typename LabelCloud::Ptr LabelCloudPtr;
typedef typename LabelCloud::ConstPtr LabelCloudConstPtr;

template <typename PointT>
class OrganizedFeatureExtractionDemoTBB
{
  public:
    typedef pcl::PointCloud<PointT> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;
    typedef pcl::PointCloud<pcl::Label> LabelCloud;
    typedef typename LabelCloud::Ptr LabelCloudPtr;
    typedef typename LabelCloud::ConstPtr LabelCloudConstPtr;

  protected:
    boost::shared_ptr<pcl::visualization::ImageViewer> image_viewer_;
    boost::shared_ptr<pcl::visualization::ImageViewer> plane_image_viewer_;
    
    bool updated_;
    bool plane_updated_;
    CloudConstPtr prev_cloud_;
    LabelCloudConstPtr prev_labels_;
    CloudConstPtr prev_plane_cloud_;
    LabelCloudConstPtr prev_plane_labels_;
    std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > prev_regions_;
    boost::mutex cloud_mutex_;
    boost::mutex plane_cloud_mutex_;
    boost::condition_variable clust_cloud_cond_;
    boost::condition_variable plane_cloud_cond_;

    // Full Planes
    boost::mutex full_planes_mutex_;
    boost::condition_variable full_planes_cv_;
    bool full_planes_updated_;
    CloudConstPtr full_planes_cloud_;
    std::vector<pcl::PointIndices> full_planes_inliers_;

    // Full Clusters
    boost::mutex full_cluster_mutex_;
    boost::condition_variable full_cluster_cv_;
    bool full_cluster_updated_;
    CloudConstPtr full_cluster_cloud_;
    std::vector<pcl::PointIndices> full_clusters_indices_;

  public:
    OrganizedFeatureExtractionDemoTBB ()
      : image_viewer_ (new pcl::visualization::ImageViewer ("Segmented Clusters")),
        plane_image_viewer_ (new pcl::visualization::ImageViewer ("Segmented Planes")),
        updated_ (false),
        plane_updated_ (false),
        prev_cloud_ (new Cloud ()),
        prev_labels_ (new LabelCloud ()),
        full_planes_cloud_ (new Cloud ()),
        full_cluster_cloud_ (new Cloud ())
    {
      image_viewer_->setPosition (0, 0);
      plane_image_viewer_->setPosition (640, 0);
    }
    
    void
    planarRegionsCallback (const CloudConstPtr cloud, boost::posix_time::ptime t, std::vector<pcl::ModelCoefficients> models, 
                           std::vector<pcl::PointIndices> inlier_indices, std::vector<pcl::PointIndices> label_indices,
                           std::vector<pcl::PointIndices> boudnary_indices)
    {
      boost::mutex::scoped_lock lock (full_planes_mutex_);
      full_planes_cloud_ = cloud;
      full_planes_inliers_ = inlier_indices;
      full_planes_updated_ = true;
      full_planes_cv_.notify_one ();
    }

    void
    fullClusterCallback (const CloudConstPtr& cloud, boost::posix_time::ptime t,
                         std::vector<pcl::PointIndices> indices)
    {
      boost::mutex::scoped_lock lock (full_cluster_mutex_);
      full_cluster_cloud_ = cloud;
      full_clusters_indices_ = indices;
      full_cluster_updated_ = true;
      full_cluster_cv_.notify_one ();
    }

    void
    clusterLabelsCallback (const CloudConstPtr& cloud, const LabelCloudConstPtr& labels)
    {
      boost::mutex::scoped_lock lock (cloud_mutex_);
      prev_cloud_ = cloud;
      prev_labels_ = labels;
      updated_ = true;
      clust_cloud_cond_.notify_one ();
    }

    void
    planeLabelsCallback (const CloudConstPtr& cloud, const LabelCloudConstPtr& labels)
    {
      boost::mutex::scoped_lock lock (plane_cloud_mutex_);
      prev_plane_cloud_ = cloud;
      prev_plane_labels_ = labels;
      plane_updated_ = true;
      plane_cloud_cond_.notify_one ();
    }
    
    // Displays segmented planes
    void
    spinVisFullPlanes ()
    {
      CloudConstPtr cloud (new Cloud ());
      LabelCloudConstPtr labels (new LabelCloud ());
      std::vector<pcl::PointIndices> inlier_inds;

      {  
        boost::mutex::scoped_lock lock (full_planes_mutex_);
        while (!full_planes_updated_)
        {
          full_planes_cv_.wait (lock);
        }
        full_planes_cloud_.swap (cloud);
        inlier_inds = full_planes_inliers_;
        full_planes_updated_ = false;
      }
      
      CloudPtr color_cloud (new Cloud (*cloud));
      unsigned char red [6] = {255,   0,   0, 255, 255,   0};
      unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
      unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};
      
      for (size_t i = 0; i < inlier_inds.size (); i++)
      {
        for (size_t j = 0; j < inlier_inds[i].indices.size (); j++)
        {
          color_cloud->points[inlier_inds[i].indices[j]].r = (cloud->points[inlier_inds[i].indices[j]].r + red[i%6]) / 2;
          color_cloud->points[inlier_inds[i].indices[j]].g = (cloud->points[inlier_inds[i].indices[j]].g + grn[i%6]) / 2;
          color_cloud->points[inlier_inds[i].indices[j]].b = (cloud->points[inlier_inds[i].indices[j]].b + blu[i%6]) / 2;
        }
      }

      if (color_cloud->points.size () > 200)
        plane_image_viewer_->addRGBImage<PointT>(color_cloud, "label_image", 0.2);
      
      plane_image_viewer_->spinOnce ();
    }

    // Displays Raw Plane Labels
    void
    spinVisPlanes ()
    {
      CloudConstPtr cloud (new Cloud ());
      LabelCloudConstPtr labels (new LabelCloud ());

      {  
        boost::mutex::scoped_lock lock (plane_cloud_mutex_);
        while (!plane_updated_)
        {
          plane_cloud_cond_.wait (lock);
        }
        prev_plane_cloud_.swap (cloud);
        prev_plane_labels_.swap (labels);
        plane_updated_ = false;
      }
      
      CloudPtr color_cloud (new Cloud (*cloud));
      unsigned char red [6] = {255,   0,   0, 255, 255,   0};
      unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
      unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};
      
      for (size_t i = 0; i < cloud->points.size (); i++)
      {
        if (labels->points[i].label == std::numeric_limits<unsigned>::max ())
        {
          // Do nothing
        }
        else
        {
          color_cloud->points[i].r = (cloud->points[i].r + red[labels->points[i].label%6]) / 2;
          color_cloud->points[i].g = (cloud->points[i].g + grn[labels->points[i].label%6]) / 2;
          color_cloud->points[i].b = (cloud->points[i].b + blu[labels->points[i].label%6]) / 2;
        }
        
      } 
      if (color_cloud->points.size () > 200)
        plane_image_viewer_->addRGBImage<PointT>(color_cloud, "label_image", 0.2);
      
      plane_image_viewer_->spinOnce ();
    }

    // Displays Raw Cluster Labels
    void
    spinVisClusters ()
    {
      CloudConstPtr cloud (new Cloud ());
      LabelCloudConstPtr labels (new LabelCloud ());

      {  
        boost::mutex::scoped_lock lock (cloud_mutex_);
        while (!updated_)
        {
          clust_cloud_cond_.wait (lock);
        }
        prev_cloud_.swap (cloud);
        prev_labels_.swap (labels);
        updated_ = false;
      }
      
      CloudPtr color_cloud (new Cloud (*cloud));
      unsigned char red [6] = {255,   0,   0, 255, 255,   0};
      unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
      unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};
      
      for (size_t i = 0; i < cloud->points.size (); i++)
      {
        if (labels->points[i].label == std::numeric_limits<unsigned>::max ())
        {
          // Do nothing
        }
        else
        {
          color_cloud->points[i].r = (cloud->points[i].r + red[labels->points[i].label%6]) / 2;
          color_cloud->points[i].g = (cloud->points[i].g + grn[labels->points[i].label%6]) / 2;
          color_cloud->points[i].b = (cloud->points[i].b + blu[labels->points[i].label%6]) / 2;
        }
        
      } 
      if (color_cloud->points.size () > 200)
        image_viewer_->addRGBImage<PointT>(color_cloud, "label_image", 0.2);
      
      image_viewer_->spinOnce ();
    }

    // Display Full Clusters
    void
    spinVisFullClusters ()
    {
      CloudConstPtr cloud (new Cloud ());
      LabelCloudConstPtr labels (new LabelCloud ());
      std::vector<pcl::PointIndices> cluster_inds;
      
      {  
        boost::mutex::scoped_lock lock (full_cluster_mutex_);
        while (!full_cluster_updated_)
        {
          full_cluster_cv_.wait (lock);
        }
        cloud.swap (full_cluster_cloud_);
        cluster_inds = full_clusters_indices_;
        full_cluster_updated_ = false;
      }

      CloudPtr color_cloud (new Cloud (*cloud));
      unsigned char red [6] = {255,   0,   0, 255, 255,   0};
      unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
      unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};

      for (size_t i = 0; i < cluster_inds.size (); i++)
      {
        for (size_t j = 0; j < cluster_inds[i].indices.size (); j++)
        {
          color_cloud->points[cluster_inds[i].indices[j]].r = (cloud->points[cluster_inds[i].indices[j]].r + red[i%6]) / 2;
          color_cloud->points[cluster_inds[i].indices[j]].g = (cloud->points[cluster_inds[i].indices[j]].g + grn[i%6]) / 2;
          color_cloud->points[cluster_inds[i].indices[j]].b = (cloud->points[cluster_inds[i].indices[j]].b + blu[i%6]) / 2;          
        }
      }
      
      if (color_cloud->points.size () > 200)
        image_viewer_->addRGBImage<PointT>(color_cloud, "label_image", 0.2);
      
      image_viewer_->spinOnce ();
    }
};

int
main (int argc, char** argv)
{
  bool raw_labels = true;
  if (pcl::console::parse_argument (argc, argv, "-raw", raw_labels) == -1)
    raw_labels = false;
  
  // Create our segmentation class
  cogrob::OrganizedSegmentationTBB<PointT> seg;

  // Create a grabber, and hook it up to the feature extraction
  pcl::OpenNIGrabber ni_grabber ("#1");
  boost::function<void (const CloudConstPtr&)> f = boost::bind (&cogrob::OrganizedSegmentationTBB<PointT>::cloudCallback, &seg, _1);
  boost::signals2::connection c = ni_grabber.registerCallback (f);

  // Hook up our demo app callbacks, which will visualize the segmentation results
  OrganizedFeatureExtractionDemoTBB<PointT> demo;
  boost::function<void(const CloudConstPtr&, const LabelCloudConstPtr&)> cluster_label_callback = boost::bind (&OrganizedFeatureExtractionDemoTBB<PointT>::clusterLabelsCallback, &demo, _1, _2);
  if (raw_labels)
    seg.setClusterLabelsCallback (cluster_label_callback);
  boost::function<void(const CloudConstPtr&, const LabelCloudConstPtr&)> plane_label_callback = boost::bind (&OrganizedFeatureExtractionDemoTBB<PointT>::planeLabelsCallback, &demo, _1, _2);
  if (raw_labels)
    seg.setPlaneLabelsCallback (plane_label_callback);

  boost::function<void(const CloudConstPtr, boost::posix_time::ptime, std::vector<pcl::ModelCoefficients>, std::vector<pcl::PointIndices>, std::vector<pcl::PointIndices>, std::vector<pcl::PointIndices>)> full_plane_callback = boost::bind (&OrganizedFeatureExtractionDemoTBB<PointT>::planarRegionsCallback, &demo, _1, _2, _3, _4, _5, _6);
  if (!raw_labels)
    seg.setFullPlanarRegionCallback (full_plane_callback);

  boost::function<void(const CloudConstPtr, boost::posix_time::ptime, std::vector<pcl::PointIndices>)> full_cluster_callback = boost::bind (&OrganizedFeatureExtractionDemoTBB<PointT>::fullClusterCallback, &demo, _1, _2, _3);
  if (!raw_labels)
    seg.setFullClusterCallback (full_cluster_callback);

  // Start spinning
  ni_grabber.start ();
  seg.spin ();

  while (true)
  {
    boost::this_thread::sleep (boost::posix_time::milliseconds (10));
    if (raw_labels)
    {  
      demo.spinVisClusters ();
      demo.spinVisPlanes ();
    }
    else
    {
      demo.spinVisFullPlanes ();
      demo.spinVisFullClusters ();
    }
  }


  return (0);
}
