#pragma once


#include <omnimapper/object_recognition.h>
#include <pcl/surface/mls.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <gtsam/geometry/Pose3.h>


template<typename PointT>
class ObjectDiscovery
{

    public:
    ObjectDiscovery();
    ~ObjectDiscovery(){};


    void createGraph();
    void loadRepresentations(std::string dir);
    void createFinalCloud(std::string dir);
    void reconstructSurface(typename pcl::PointCloud<PointT>::Ptr merged, int id);
    float  computeJaccardIndex(
            Eigen::Vector4f min_pt_1, Eigen::Vector4f min_pt_2,
                    Eigen::Vector4f max_pt_1, Eigen::Vector4f max_pt_2);
    void mergeClouds();
    int findLabel(int segment);
    int findMin(int seg, std::vector<int> seg_vec);
    std::map<int, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> map_cloud;

    protected:
    int max_object_size, max_current_size;


      boost::shared_ptr<ObjectRecognition<pcl::SHOT1344> > correspondence_estimator;
  	std::map<int, std::vector<int> > graph;
  	std::map<int, std::vector<int> > match_graph;
    std::vector<int> segment_arr;
    std::string object_dir_;


};


