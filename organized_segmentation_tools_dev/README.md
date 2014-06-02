organized_segmentation_tools_dev
================================

### Description

Organized Segmentation Tools is a utility for segmenting organized point clouds, as produced by an RGB-D sensor.  This includes plane segmentation, euclidean clustering, and edge detection.  The provided library enables these to be run in a multi-threaded pipeline using Intel's Threading Building Blocks (TBB).  For example, normal estimation, plane segmentation, and clustering may be run concurrently.  The project builds both a shared library suitable for use in other projects, and an example application demonstrating the tools as applied to a live stream from an RGB-D sensor.

### Build

* `mkdir build`
* `cd build`
* `cmake ..`
* `make`

### Run

`./organized_segmentation_tbb_demo`

### References

The following papers describe the techniques used here:

[Efficient Organized Point Cloud Segmentation with Connected Components](http://www.cc.gatech.edu/~atrevor/resources/publications/spme_2013_segmentation.pdf)

 `@inproceedings{trevor2013efficient,
  title={Efficient Organized Point Cloud Segmentation with Connected Components},
  author={Trevor, A and Gedikli, Suat and Rusu, R and Christensen, H},
  booktitle={3rd Workshop on Semantic Perception Mapping and Exploration (SPME), Karlsruhe, Germany},
  year={2013}
}`

[RGB-D edge detection and edge-based registration](http://www.cc.gatech.edu/~cchoi/Publications_files/Choi13iros_edge.pdf)

`@inproceedings{choi2013rgb,
  title={RGB-D edge detection and edge-based registration},
  author={Choi, Changhyun and Trevor, Alexander JB and Christensen, Henrik I},
  booktitle={Intelligent Robots and Systems (IROS), 2013 IEEE/RSJ International Conference on},
  pages={1568--1575},
  year={2013},
  organization={IEEE}
}`

[Adaptive neighborhood selection for real-time surface normal estimation from organized point cloud data using integral images](http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=6385999&tag=1)

 `@inproceedings{holzer2012adaptive,
  title={Adaptive neighborhood selection for real-time surface normal estimation from organized point cloud data using integral images},
  author={Holzer, Stefan and Rusu, Radu Bogdan and Dixon, M and Gedikli, Suat and Navab, Nassir},
  booktitle={Intelligent Robots and Systems (IROS), 2012 IEEE/RSJ International Conference on},
  pages={2684--2689},
  year={2012},
  organization={IEEE}
}`
