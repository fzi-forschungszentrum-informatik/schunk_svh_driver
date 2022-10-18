
---

# ++ Deprecation Notice ++

This repository has been split and refactured into two components:

- A plain C++ library: [schunk_svh_library](https://github.com/fzi-forschungszentrum-informatik/schunk_svh_library)
- A ROS1/ROS2 wrapper: [schunk_svh_ros_driver](https://github.com/fzi-forschungszentrum-informatik/schunk_svh_ros_driver)

It's now significantly easier to build and use this driver in all mayor ROS1/ROS2 versions and in custom, non-ROS projects.

Please continue to open issues and pull requests there as usual.

---

# Schunk SVH driver
[![Build Status](https://travis-ci.org/fzi-forschungszentrum-informatik/schunk_svh_driver.svg?branch=master)](https://travis-ci.org/fzi-forschungszentrum-informatik/schunk_svh_driver)


This is the driver package for the SCHUNK SVH Five-Finger-Hand.
It was developed on behalf of SCHUNK Mobile Greifsysteme GmbH, Lauffen/Neckar, Germany
at the FZI Research Center for Information Technology in Karlsruhe, Germany.

The package contains the following core components:
- The low level hardware driver
- The ROS abstraction layer
- 3D model and kinematics description for visualization and grasp planning
These main components are distributed under a LGPL license.

Furthermore this package contains a build system and two libraries that are not
part of the SVH Driver but which are needed to build it:
- icmaker  (BSD License)
- icl_core (LGPL License)
- icl_comm (LGPL License)
These components were independently developed at the
FZI Research Center for Information Technology in Karlsruhe, Germany.

Also included is the TinyXML Library which was developed by
Lee Thomason (http://www.grinninglizard.com)
and which ships unter the zlib/libpng license.


See license folder for the license texts.
