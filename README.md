# Omnimapper

![image](https://user-images.githubusercontent.com/2293573/86566424-791ba380-bf1e-11ea-87b3-fe3ca8b35de8.png)

Simultaneous Localization and Mapping (SLAM) is not a problem with a one-size-fits-all solution. The literature includes a variety of SLAM approaches targeted at different environments, platforms, sensors, CPU budgets, and applications. We propose OmniMapper, a modular multimodal framework and toolbox for solving SLAM problems. The system can be used to generate pose graphs, do feature-based SLAM, and also includes tools for semantic mapping. Multiple measurement types from different sensors can be combined for multimodal mapping. It is open with standard interfaces to allow easy integration of new sensors and feature types. We present a detailed description of the mapping approach, as well as a software framework that implements this, and present detailed descriptions of its applications to several domains including mapping with a service robot in an indoor environment, large- scale mapping on a PackBot, and mapping with a handheld RGBD camera.

## Publications

We kindly ask to cite our paper if you find this library useful:

> A. J. B. Trevor, J. G. Rogers and H. I. Christensen, "OmniMapper: A modular multimodal mapping framework," 2014 IEEE International Conference on Robotics and Automation (ICRA), Hong Kong, 2014, pp. 1983-1990, doi:[ 10.1109/ICRA.2014.6907122](https://doi.org/10.1109/ICRA.2014.6907122).

```
@INPROCEEDINGS{6907122,
  author={A. J. B. {Trevor} and J. G. {Rogers} and H. I. {Christensen}},
  booktitle={2014 IEEE International Conference on Robotics and Automation (ICRA)}, 
  title={OmniMapper: A modular multimodal mapping framework}, 
  year={2014},
  volume={},
  number={},
  pages={1983-1990},
  keywords={cameras;control engineering computing;image colour analysis;pose estimation;robot vision;SLAM (robots);OmniMapper framework;modular multimodal mapping framework;simultaneous localization and mapping;SLAM approach;pose graph generation;feature-based SLAM;software framework;PackBot;handheld RGBD camera;red-green-blue-depth camera;Simultaneous localization and mapping;Time measurement;Three-dimensional displays;Trajectory},
  doi={10.1109/ICRA.2014.6907122},
  ISSN={1050-4729},
  month={May},}
```

## Demo

[![image](https://user-images.githubusercontent.com/2293573/86566926-5473fb80-bf1f-11ea-91a8-dceda2382fa4.png)](https://www.youtube.com/watch?v=djLKmDMsdxM)

> https://www.youtube.com/watch?v=djLKmDMsdxM

## Installation

See Dockerfile within respective ROS wrapper repostory for detailed build instructions.

* https://github.com/CogRob/omnimapper_ros
