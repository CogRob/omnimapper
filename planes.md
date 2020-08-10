## Planes for Mapping

A type of landmark used by our system is planar surfaces extracted
from point cloud data. This type of feature is of particular relevance
to service robots, as humans tend to place objects on surfaces such
as tables, shelves, and counters. Planar features corresponding to
walls also give useful information regarding the structure and
boundaries of spaces, such as rooms or hallways. In addition to
serving as landmarks for SLAM, our system allows users to label
planar surfaces, for reference in later commands.

3D laser scanners or RGB-D sensors such as the Intel Realsense camera
can be used to collect suitable data. Planes are then extracted from
the point cloud by an iterative RANdom SAmple Consensus (RANSAC)
method, which allows us to find all planes meeting constraints for
size and number of inliers. A clustering step is performed on
extracted planes separate multiple coplanar surfaces, such as two
tables with the same height, but at different locations. We make use
of the Point Cloud Library (PCL) for much of our point cloud
processing.  Planes can be represented by the well known equation:

ax + by + cz + d = 0.

Our mapper then represents the planes as: p = {n, hull} where: n = {a,
b, c, d} and hull is a point cloud of the vertices of the plane’s
convex hull. As the robot platform moves through the environment and
measures planes multiple times, the mapped planes’ hulls will be
extended with each new portion seen, allowing the use of large planes
such as walls where the full extent is typically not observed in any
single measurement.

This type of plane can then be used for localization purposes by using
the surface normal and perpendicular distance from the robot. In
addition to being useful for localization, we believe that these
surfaces are also useful for communication with humans. Many service
robot tasks may require interaction with objects on horizontal planar
surfaces, such as tables or shelves, and navigational tasks may
require an understanding of planar surfaces such as walls or doors. In
order to support such tasks, our mapping system allows planar surfaces
to optionally support a label, such as "kitchen table" or "Henrik’s
desk," so that they may be easily referenced by a human user. Labels
are entered interactively via a command line application. Planes
corresponding to walls, the floor, or the ceiling can also be labeled,
and multiple planes can share the same label as well. For example, one
could label two walls of a hallway as "front hall," which gives the
robot an idea of the extent of this structure.

Preliminary results on maps that represent the locations and extent of
this type of planar feature have been reported in multiple of our
papers. More recent work includes the use of these features as
landmarks for OmniMapper. 1, and a top-down view is shown in Figure
2 after a large correction was made following a loop-closure. A
close-up view of some labeled planar features is shown in Figure 16.
Figure 6: An example of a map composed of planar surfaces. The mapped
area shows several cubicles, with walls, cubicle walls, and desks used
as landmarks. The convex hulls of the planar regions are shown in red,
and blue lines represent measurements, indicating which poses features
were measured from. Surface normals are shown in red for vertical
surfaces, and green for horizontal surfaces. The full point clouds are
displayed in white, for visualization purposes only; only the
extracted planes are used for mapping and localization.

## Code Integration

OmniMapper is organized around basic point based data that are
integrated into a factor graph. The integration of different features
is achieved using plug-ins. A plug-in does data segmentation and
feature matching (data association). The plug-in template is provided
in the src/plugins directory. The plugin directory has support for
integration of infinite plans, matching of ICP points, simple pose
integration (from robot pose estimates) and handling of bounded planes
(as described above)

The omnimapper directory has files for integration of the robot base,
infinite planes, bounded planes and simple factors. It also has the
integration of factors for the factor graph and handling of
transformations between different features. The subdirectory organized
segmentation contains the code for segmentation of point clouds into
planar segments using PCL. 
