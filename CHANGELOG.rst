^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package schunk_svh_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2017-04-25)
------------------
* Added hardware support for the 2nd hardware version of the Schunk SVH
* Added dynamic parameter switcher for automatically setting parameters
* Improved sine test with increased movement range, thumb movement and possibility to change the speed
* Extracted ``fzi_icl_core`` and ``fzi_icl_comm``
* Fixed xacro warnings
* Contributors: Felix Mauch, Pascal Becker, Johannes Mangler

0.1.7 (2015-06-18)
------------------
* Added correct Reset of package counts
* Added Retry functionality for connection failures
* Added Readout of efforts
* Correctly use the given homing settings ( fixes #1 )
* Added safety warning for high current settings
* Added much debugging output for the reset routine
* Fixes of typos, comments and other minor things
* Rudimentary windows support for icl libs
* Removed icl_comm_websocket as it was not strictly needed for the ROSnode of the driver
* Added effort feedback
* Added channel current output
* Added retry option for the connect routine to make autostart more robust
* Added the TcoNoDelay transport hint
* Include name prefix for the channel names
* add collision meshes to URDF
* Removed parts that where actually from another project and did not belong into the SVH Driver package
* Added a name prefix for the hand to allow multiple instantiation of the hand model.
* Make hand member names dependent of hand name
* Added a mainpage.dox with an image and the corresponing rosdoc config
* Contributors: Andreas Hermann, Georg Heppner, Nils Berg, Steffen Ruehl

0.1.6 (2014-09-30)
------------------
* Added a mainpage.dox with an image and the corresponing rosdoc config
* Fixed xacro and run_demo to work with the new package structure
* Added a new node only launch script
* Contributors: Georg Heppner

0.1.5 (2014-09-29)
------------------
* Moved rviz config to etc and changed gui to a default no show
* Fixed a counting variable which was comparing signed with unsigned
* Changed the way to install udev rules to be a script for devel and a rosrun for install targets
* Enabled the svh_sin_test node by default as this is easier to start
* Changed the logging mechanism to use ~/.ros/log instead of the project folder as this is not writable in installs
  Also changed many default values and how the config values are read to provide better user experience
* Added a queue size to the topics published by the plugin as it is required by hydro
* Contributors: Georg Heppner

0.1.4 (2014-09-26)
------------------
* Removed preliminary tests that remained from development and where not meant to be published
* Contributors: Georg Heppner

0.1.3 (2014-09-26)
------------------
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
