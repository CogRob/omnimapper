#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/make.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/register/ring.hpp>
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/multi/multi.hpp>
#include <boost/geometry/strategies/strategies.hpp>

typedef std::vector<pcl::PointXYZRGBA,
                    Eigen::aligned_allocator<pcl::PointXYZRGBA> >
    PointVector;
BOOST_GEOMETRY_REGISTER_POINT_2D(pcl::PointXYZRGBA, float,
                                 boost::geometry::cs::cartesian, x, y);
BOOST_GEOMETRY_REGISTER_RING(PointVector);

namespace omnimapper {
template <typename PointT>
bool fusePlanarPolygonsXY(const pcl::PointCloud<PointT>& poly1,
                          const pcl::PointCloud<PointT>& poly2,
                          pcl::PointCloud<PointT>& poly_out);

template <typename PointT>
bool fusePlanarPolygonsConvexXY(const pcl::PointCloud<PointT>& poly1,
                                const pcl::PointCloud<PointT>& poly2,
                                pcl::PointCloud<PointT>& poly_out);
}  // namespace omnimapper
