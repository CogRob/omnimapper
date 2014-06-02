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
#include <pcl/segmentation/plane_refinement_comparator.h>
#include <tbb/task.h>
#include <tbb/pipeline.h>
#include <tbb/task_scheduler_init.h>
#include <tbb/spin_mutex.h>
#include <tbb/tbb_thread.h>
#include <tbb/task_group.h>
#include <tbb/parallel_invoke.h>

namespace cogrob
{
  template <typename PointT>
  OrganizedSegmentationTBB<PointT>::OrganizedSegmentationTBB () 
    : input_cloud_ (boost::none),
      ne_output_normals_ (boost::none),
      ne_output_cloud_ (boost::none),
      mps_input_cloud_ (boost::none),
      mps_input_normals_ (boost::none),
      mps_output_regions_ (boost::none),
      mps_output_cloud_ (boost::none),
      mps_output_labels_ (new LabelCloud ()),
      clust_input_cloud_ (boost::none),
      clust_input_labels_ (boost::none),
      pub_cluster_labels_ (boost::none),
      pub_cluster_cloud_ (boost::none),
      pub_occluding_edge_cloud_ (boost::none),
      pub_mps_regions_ (boost::none),
      clust_output_labels_ (boost::none),
      oed_output_occluding_edge_cloud_ (boost::none),
      updated_cloud_ (false),
      ne_(new pcl::IntegralImageNormalEstimation<PointT, pcl::Normal>()),
      mps_(new pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label>()),
      euclidean_cluster_comparator_ (new pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>()),
      min_plane_inliers_ (10000),
      min_cluster_inliers_ (1000),
      debug_ (false),
      timing_ (false),
      ready_ (true)
  {
    if (debug_)
      printf ("Start!\n");
    
    // Set up Normal Estimation
    //ne_->setNormalEstimationMethod (ne_->SIMPLE_3D_GRADIENT);
    ne_->setNormalEstimationMethod (ne_->COVARIANCE_MATRIX);
    ne_->setMaxDepthChangeFactor (0.02f);
    ne_->setNormalSmoothingSize (20.0f);
    
    // Set up plane segmentation
    mps_->setMinInliers (min_plane_inliers_);
    mps_->setAngularThreshold (pcl::deg2rad (2.0));//2.0
    mps_->setDistanceThreshold (0.02);//0.03
    mps_->setProjectPoints (true);
    //mps_->setRemoveDuplicatePoints (false); // need atrevor pcl branch
    pcl::PlaneCoefficientComparator<pcl::PointXYZRGBA, pcl::Normal>::Ptr plane_compare (new pcl::PlaneCoefficientComparator<pcl::PointXYZRGBA, pcl::Normal>());
    plane_compare->setAngularThreshold (pcl::deg2rad (2.0));//3.0
    plane_compare->setDistanceThreshold (0.01, true);//0.02, true
    mps_->setComparator (plane_compare);
    
    pcl::PlaneRefinementComparator<pcl::PointXYZRGBA, pcl::Normal, pcl::Label>::Ptr refine_compare (new pcl::PlaneRefinementComparator<pcl::PointXYZRGBA, pcl::Normal, pcl::Label> ());
    refine_compare->setDistanceThreshold (0.005, false);//0.01, true//0.0025
    mps_->setRefinementComparator (refine_compare);
    
    // Set up edge detection
    #ifdef HAVE_ORGANIZED_EDGES
    oed.setDepthDisconThreshold (0.04f);
    oed.setMaxSearchNeighbors (100);
    oed.setEdgeType (oed.EDGELABEL_NAN_BOUNDARY | oed.EDGELABEL_OCCLUDING | oed.EDGELABEL_OCCLUDED);
    //oed.setEdgeType (oed.EDGELABEL_RGB_CANNY);
    #endif

    // Set up output
    if (timing_)
    {
      ne_times_file_.open ("/home/atrevor/Desktop/ne_times.txt");
      mps_times_file_.open ("/home/atrevor/Desktop/mps_times.txt");
    }

      //PCL_INFO ("Starting process thread\n");
      //process_thread = boost::thread (&OrganizedSegmentationTBB::spin, this);
  }
    
  // Get latest cloud from the sensor
  template <typename PointT> void 
  OrganizedSegmentationTBB<PointT>::cloudCallback (const CloudConstPtr& cloud)
  {
    // Store cloud
    boost::lock_guard<boost::mutex> lock (cloud_mutex);
    {
      prev_sensor_cloud_ = cloud;
      updated_cloud_ = true;
      ready_ = false;
    }
    //updated_cond_.notify_one ();
  }
  
  template <typename PointT> bool
  OrganizedSegmentationTBB<PointT>::ready ()
  {
    boost::mutex::scoped_lock lock(cloud_mutex);
    return (ready_);
  }
  

  template <typename PointT> void
  OrganizedSegmentationTBB<PointT>::spin ()
  {
    spin_thread = boost::thread (&OrganizedSegmentationTBB<PointT>::spinThread, this);
  }
  

  template <typename PointT> void
  OrganizedSegmentationTBB<PointT>::spinThread ()
  {
    while (true)
    {
      boost::this_thread::sleep (boost::posix_time::milliseconds (5));
      spinOnce ();
    }
  }

    template <typename PointT> void
    OrganizedSegmentationTBB<PointT>::spinOnce()
    {
      double frame_start = pcl::getTime ();
      // Get latest cloud
      input_cloud_ = boost::none;
      if (cloud_mutex.try_lock()) 
      {
        if (updated_cloud_)
        {
          input_cloud_ = prev_sensor_cloud_;
          updated_cloud_ = false;
          ready_ = false;
        }
        cloud_mutex.unlock();
      }
      
      // Normal Estimation && MPS && Clustering
      tbb::task_group group;
      group.run ([&] { computeNormals (); });
      group.run ([&] { computePlanes (); });
      if ((cluster_cloud_callbacks_.size () > 0) || (cluster_label_cloud_callbacks_.size () > 0) || (full_cluster_callbacks_.size () > 0))
        group.run ([&] { computeClusters (); });
#ifdef HAVE_ORGANIZED_EDGE
      if (occluding_edge_callback_)
        group.run ([&] { computeEdges (); });
#endif
      group.run ([&] { publish (); });
      group.wait ();

      // Move clouds along pipeline
      pub_cluster_cloud_ = clust_input_cloud_;
      pub_cluster_labels_ = clust_output_labels_;
      pub_clusters_ = clust_output_clusters_;
      pub_cluster_indices_ = clust_output_cluster_indices_;
      pub_occluding_edge_cloud_ = oed_output_occluding_edge_cloud_;
      pub_mps_regions_ = mps_output_regions_;

      clust_input_cloud_ = mps_input_cloud_;
      clust_input_regions_ = mps_output_regions_;
      clust_input_labels_ = mps_output_labels_;
      clust_input_model_coefficients_ = mps_output_model_coefficients_;
      clust_input_inlier_indices_ = mps_output_inlier_indices_;
      clust_input_label_indices_ = mps_output_label_indices_;
      clust_input_boundary_indices_ = mps_output_boundary_indices_;

      mps_input_normals_ = ne_output_normals_;
      mps_input_cloud_ = input_cloud_;

      if(!ne_output_normals_ && !mps_output_labels_ && !clust_output_labels_ && !pub_cluster_labels_ && !oed_output_occluding_edge_cloud_ && !pub_occluding_edge_cloud_)
      {
        boost::mutex::scoped_lock (cloud_mutex);
        {
          ready_ = true;
        }
      }

      double frame_end = pcl::getTime ();
      if (debug_)
        std::cout << "Frame took: " << double(frame_end - frame_start) << std::endl;
    }

  template <typename PointT> void
  OrganizedSegmentationTBB<PointT>::publish ()
  {
    // Publish plane Labels
    if (plane_label_cloud_callback_)
    {
      if (clust_input_cloud_ && clust_input_labels_)
      {
        plane_label_cloud_callback_ (*clust_input_cloud_, *clust_input_labels_);
      }      
    }

    // Publish plane regions
    if (planar_region_stamped_callbacks_.size () > 0)
    {
      if (pub_mps_regions_)
      {
        Time timestamp = stamp2ptime ((*clust_input_cloud_)->header.stamp);
        for (int i = 0; i < planar_region_stamped_callbacks_.size (); i++)
          planar_region_stamped_callbacks_[i](*pub_mps_regions_, timestamp);
      }
    }

    // Publish full planar regions
    if (full_planar_callbacks_.size () > 0)
    {
      if (pub_mps_regions_)
      {
        Time timestamp = stamp2ptime ((*clust_input_cloud_)->header.stamp);
        for (int i = 0; i < full_planar_callbacks_.size (); i++)
          full_planar_callbacks_[i](*clust_input_cloud_, timestamp, 
                                    *clust_input_model_coefficients_, 
                                    *clust_input_inlier_indices_, 
                                    *clust_input_label_indices_, 
                                    *clust_input_boundary_indices_);
      }
    }

    // Publish Cluster Labels
    if (cluster_label_cloud_callbacks_.size () > 0)
    {
      if (pub_cluster_cloud_ && pub_cluster_labels_)
      {  
        for (int i = 0; i < cluster_label_cloud_callbacks_.size (); i++)
          cluster_label_cloud_callbacks_[i](*pub_cluster_cloud_, *pub_cluster_labels_);
      }
    }
    
    // Publish Cluster Clouds
    if (cluster_cloud_callbacks_.size () > 0)
    {
      if (pub_cluster_cloud_)
      {
        Time timestamp = stamp2ptime ((*pub_cluster_cloud_)->header.stamp);
        for (int i = 0; i < cluster_cloud_callbacks_.size (); i++)
        {
          cluster_cloud_callbacks_[i] (*pub_clusters_, timestamp, *pub_cluster_indices_);
        }
      }
    }

    // Publish full cluster info
    if (full_cluster_callbacks_.size () > 0)
    {
      if (pub_cluster_cloud_ && pub_cluster_indices_)
      {
        Time timestamp = stamp2ptime ((*pub_cluster_cloud_)->header.stamp);
        for (int i = 0; i < full_cluster_callbacks_.size (); i++)
        {
          full_cluster_callbacks_[i] (*pub_cluster_cloud_, timestamp, *pub_cluster_indices_);
        }
      }
    }

    // Publish Occluding Edges
    if (occluding_edge_callback_)
    {
      if (pub_occluding_edge_cloud_)
      {
        occluding_edge_callback_ (*pub_occluding_edge_cloud_);
      }
    }
    
    
  }
    
    // Compute the normals
  template <typename PointT> void
  OrganizedSegmentationTBB<PointT>::computeNormals ()
  {
    if (!input_cloud_)
    {
      ne_output_cloud_ = boost::none;
      ne_output_normals_ = boost::none;
      return;
    }
    ne_->setInputCloud(*input_cloud_);
    double start = pcl::getTime ();
    NormalCloudPtr normals (new NormalCloud ());
    
    ne_->compute (*normals);
    ne_output_normals_ = normals;
    double end = pcl::getTime ();
    if (timing_)
    {
      ne_times_file_ << double(end - start) << std::endl;
      std::cout << double(end - start) << std::endl;
    }
    if (debug_)
      std::cout << "NE took : " << double(end - start) << std::endl;
    return;
  }

  // Compute clusters
  template <typename PointT> void
  OrganizedSegmentationTBB<PointT>::computeClusters ()
  {
    if (!(clust_input_labels_ && clust_input_cloud_))
      return;
    //Segment Objects
    std::vector<CloudPtr> output_clusters;
    std::vector<pcl::PointIndices> output_cluster_indices;
    clust_output_clusters_ = output_clusters;
    clust_output_cluster_indices_ =  output_cluster_indices;

    clust_output_labels_ = LabelCloudPtr(new LabelCloud ());

    std::vector<bool> plane_labels;
    plane_labels.resize (clust_input_label_indices_->size (), false);
    
    if (clust_input_regions_->size () > 0)
    {
      for (size_t i = 0; i < clust_input_label_indices_->size (); i++)
      {
        if ((*clust_input_label_indices_)[i].indices.size () > min_plane_inliers_)
        {
          plane_labels[i] = true;
        }
      }  
    }
    
    euclidean_cluster_comparator_->setInputCloud (*clust_input_cloud_);
    euclidean_cluster_comparator_->setLabels (*clust_input_labels_);
    euclidean_cluster_comparator_->setExcludeLabels (plane_labels);
    euclidean_cluster_comparator_->setDistanceThreshold (0.01f, false);
      
    pcl::PointCloud<pcl::Label> euclidean_labels;
    std::vector<pcl::PointIndices> euclidean_label_indices;
    pcl::OrganizedConnectedComponentSegmentation<PointT,pcl::Label> euclidean_segmentation (euclidean_cluster_comparator_);
    euclidean_segmentation.setInputCloud (*clust_input_cloud_);
    euclidean_segmentation.segment (*(*clust_output_labels_), euclidean_label_indices);

    for (size_t i = 0; i < euclidean_label_indices.size (); i++)
    {
      if (euclidean_label_indices[i].indices.size () > 1000)
      {
        CloudPtr cluster (new Cloud ());
        pcl::copyPointCloud (*(*clust_input_cloud_), euclidean_label_indices[i].indices, *cluster);
        
        clust_output_clusters_->push_back (cluster);
        clust_output_cluster_indices_->push_back(euclidean_label_indices[i]);
        }    
    }
  }
  
  
  // Compute planes
  template <typename PointT> void
  OrganizedSegmentationTBB<PointT>::computePlanes ()
  {
    //std::cout << "in compute planes" << std::endl;
    
    if ((!mps_input_cloud_) || (!mps_input_normals_))
    {
      mps_output_labels_ = boost::none;
      mps_output_regions_ = boost::none;
      return;
    }
    if ((*mps_input_cloud_)->points.size () == 0)
    {
      mps_output_labels_ = boost::none;
      mps_output_regions_ = boost::none;
      return;
    }
    
    mps_->setInputNormals (*mps_input_normals_);
    mps_->setInputCloud (*mps_input_cloud_);
    
    double init_start = pcl::getTime ();
    std::vector<pcl::ModelCoefficients> model_coefficients;
    std::vector<pcl::PointIndices> inlier_indices;  
    std::vector<pcl::PointIndices> label_indices;
    std::vector<pcl::PointIndices> boundary_indices;
    mps_output_model_coefficients_ = model_coefficients;
    mps_output_inlier_indices_ = inlier_indices;
    mps_output_label_indices_ = label_indices;
    mps_output_boundary_indices_ = boundary_indices;
    mps_output_labels_ = LabelCloudPtr(new LabelCloud ());
    mps_output_regions_ = std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > ();
 
    double start = pcl::getTime ();
    mps_->segmentAndRefine ((*mps_output_regions_), (*mps_output_model_coefficients_), (*mps_output_inlier_indices_), (*mps_output_labels_), (*mps_output_label_indices_), (*mps_output_boundary_indices_));

    double end = pcl::getTime ();
    if (debug_)
      std::cout << "mps segment and refine took: " << double(end - start) << std::endl;

    if (timing_)
    {
      mps_times_file_ << double(end - start) << std::endl;
      std::cout << double(end - start) << std::endl;
    }
  }

  // Extract edges
#ifdef HAVE_ORGANIZED_EDGE
  template <typename PointT> void
  OrganizedSegmentationTBB<PointT>::computeEdges ()
  {
    if (!input_cloud_)
    {
      oed_output_occluding_edge_cloud_ = boost::none;
      return;
    }
    
    double edge_start = pcl::getTime ();
    pcl::PointCloud<pcl::Label> labels;
    std::vector<pcl::PointIndices> label_indices;
    oed.setInputCloud (*input_cloud_);
    
    oed.compute (labels, label_indices);
    double edge_end = pcl::getTime ();
    if (debug_)
      std::cout << "edges took: " << double (edge_end - edge_start) << std::endl;
    //oed_output_occluding_edge_cloud_ = CloudPtr(new Cloud ());
    CloudPtr edge_cloud (new Cloud ());
    pcl::copyPointCloud (*(*input_cloud_), label_indices[1], *edge_cloud);
    oed_output_occluding_edge_cloud_ = edge_cloud;
  }
#endif
 
    template <typename PointT> void
    OrganizedSegmentationTBB<PointT>::setPlanarRegionCallback (boost::function<void (std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >&)>& fn)
    {
      planar_region_callback_ = fn;
    }

    template <typename PointT> void
    OrganizedSegmentationTBB<PointT>::setPlanarRegionStampedCallback (boost::function<void (std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >, Time)>& fn)
    {
      planar_region_stamped_callbacks_.push_back (fn);
    }

    template <typename PointT> void
    OrganizedSegmentationTBB<PointT>::setFullPlanarRegionCallback (boost::function<void(const CloudConstPtr, Time, std::vector<pcl::ModelCoefficients>, std::vector<pcl::PointIndices>, std::vector<pcl::PointIndices>, std::vector<pcl::PointIndices>)>& fn)
    {
      full_planar_callbacks_.push_back (fn);
    }

    template <typename PointT> void
    OrganizedSegmentationTBB<PointT>::setOccludingEdgeCallback (boost::function<void (const CloudConstPtr&)>& fn)
    {
      occluding_edge_callback_ = fn;
    }

  template <typename PointT> void
  OrganizedSegmentationTBB<PointT>::setPlaneLabelsCallback (boost::function<void (const CloudConstPtr&, const LabelCloudConstPtr&)>& fn)
  {
    plane_label_cloud_callback_ = fn;
  }

  template <typename PointT> void
  OrganizedSegmentationTBB<PointT>::setClusterLabelsCallback (boost::function<void (const CloudConstPtr&, const LabelCloudConstPtr&)>& fn)
  {
    cluster_label_cloud_callbacks_.push_back(fn);
  }

  template <typename PointT> void
  OrganizedSegmentationTBB<PointT>::setRegionCloudCallback (boost::function<void(const CloudConstPtr&, std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >&)>& fn)
  {
    region_cloud_callback_ = fn;
  }

  template <typename PointT> void
  OrganizedSegmentationTBB<PointT>::setClusterCloudCallback (boost::function<void(std::vector<CloudPtr>, Time,  boost::optional<std::vector<pcl::PointIndices> > )> fn)
  {
    cluster_cloud_callbacks_.push_back (fn);
  }

  template <typename PointT> void
  OrganizedSegmentationTBB<PointT>::setFullClusterCallback (boost::function<void(const CloudConstPtr, Time, std::vector<pcl::PointIndices>)>& fn)
  {
    full_cluster_callbacks_.push_back (fn);
  }
  
  template <typename PointT> boost::posix_time::ptime
  OrganizedSegmentationTBB<PointT>::stamp2ptime (uint64_t  stamp)
  {
    boost::posix_time::ptime time_t_epoch(boost::gregorian::date(1970,1,1));
    return (time_t_epoch + boost::posix_time::microseconds (stamp));
  }
    
}

//Instantiate
template class cogrob::OrganizedSegmentationTBB<pcl::PointXYZRGBA>;
