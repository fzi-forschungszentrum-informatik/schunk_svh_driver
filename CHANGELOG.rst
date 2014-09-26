^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package schunk_svh_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.4 (2014-09-26)
------------------
* Removed preliminary tests that remained from development and where not meant to be published
* Contributors: Georg Heppner

0.1.3 (2014-09-26)
-----------
* Moved the widget gui to a resource folder as it is suggested by the rqt tutorials
* Changed the includes from using find scripts to use internally set variables to prevent failures in case of multiple icl workspace instances
* Added missing plugin.xml to install targets
* Renamed include dir to adhere to currect naming schema
* Added install directives for launch files, config files and headers, udev ruels, helper files and boilderplate needed for execution
* Disabled Rotation for log files
* Fixed install directives
* Contributors: Georg Heppner

0.1.2 (2014-09-26)
------------------
* Fixed install directives in CMake
* Contributors: Andreas Hermann

0.1.1 (2014-09-26)
------------------
* Made the launch file a little bit better to support simulation without standalone flag
* Updated the quick commands to contain speeds and efforts as this is required by the joint state publisher
* Added UDEV rules to the package
* Fixed package name
* Contributors: Andreas Hermann, Georg Heppner

0.1.0 (2014-09-25)
------------------
* First public release: Contains Low-Level Driver, 3D-Model and ROS-Node
* Contributors: Georg Heppner, Lars Pfotzer, Nils Berg, Andreas Hermann, Steffen Rühl

0.0.X (2014-08-01)
------------------
* Internal Development of Low-Level-Driver
* Contributors: Georg Heppner, Lars Pfotzer, Nils Berg, Andreas Hermann, Steffen Rühl
