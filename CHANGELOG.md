# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.0.6] - 2020-07-06

### Added

- Documentation
  - Citations and setup/demo links

### Fixed

- Build system
  - Build for Ubuntu 20.04
  - Update for newer PCL API

## [0.0.5] - 2020-04-02

### Added

- Plugins
  - Bounded Planes

### Fixed

- Cmake
  - Finding TBB library
  - Missing boost component
- GTSAM
  - Update planes factor value type
- Omnimapper
  - Size check for removeDuplicatePoints

### Removed

- OpenNI dependency

## [0.0.4] - 2020-02-15

### Changed

- Refactor project
  - Re-organize headers and source layout
  - Combine organized_segmentation library
  - Use build to ament and colcon
  - Clean up old cmake

## [0.0.3] - 2019-11-17

### Changed

- Code Style
  - Adopt clang format
- GTSAM
  - Target GTSAM v4.0

### Fixed

- Cmake
  - GTSAM header setup
- Header files
  - Use of boost::posix_timer
- Build warnings
  - Build for Ubuntu 18.04

### Removed

- Old Dockerfiles
- ROS1 wrapper
  - migrated to https://github.com/CogRob/omnimapper_ros

## [0.0.2] - 2014-07-10

### Added

- Docker Support
  - New Dockerfile
- Documentation
  - New tutorials

### Changed

- Patched build
  - Updated for Ubuntu 14.04

### Fixed

- Build system
  - CSM source checkout

## [0.0.1] - 2014-06-01

### Added

- Initial release
- C++ library for pose graph SLAM
  - Plugins
    - Iterative Closest Point (ICP)
    - No motion pose
  - Utility libraries
    - Organized Segmentation Tools

[unreleased]: https://github.com/CogRob/omnimapper/releases/tag/v0.0.6...HEAD
[0.0.6]: https://github.com/CogRob/omnimapper/releases/tag/v0.0.5...v0.0.6
[0.0.5]: https://github.com/CogRob/omnimapper/releases/tag/v0.0.4...v0.0.5
[0.0.4]: https://github.com/CogRob/omnimapper/releases/tag/v0.0.3...v0.0.4
[0.0.3]: https://github.com/CogRob/omnimapper/releases/tag/v0.0.2...v0.0.3
[0.0.2]: https://github.com/CogRob/omnimapper/releases/tag/v0.0.1...v0.0.2
[0.0.1]: https://github.com/CogRob/omnimapper/releases/tag/v0.0.1
