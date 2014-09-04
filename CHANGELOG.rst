^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package schunk_svh
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Minor change to not copy the Node object without any need.
  -> Also the memory failures are fixed (was a problem within the icl)
* Merge branch 'master' of idsgit.fzi.de:ros/ros_s5fh into merging
* Renamed the Node to avoid ambiguities with the low level controller
  Moved functionality into class structure to get rid of global variables
  NOTE: Currently there is a segfault when node is closed.
  This results from an invalid oder double freeing proably even inside
  the Finger Manager and will probably also leak memory.
  This does not impede the working functionality and will be fixed within this week.
* Merge remote-tracking branch 'origin/master'
* add instructions to sed script
* add a tiny ambient part to model materials, since RViz will turn them grey otherwise
* more changes to launch files
* Also renamed the launch files. Seems to be working except for the robot modell and tfs
* Renamed the descriptions as well (untestet). Package name is now
  svh_driver
* Renamed most of the files and hopefully all of the occurences within the source to
  SVH instead of S5FH
* Changed a slight error in multiplication factor.
* Switched from enable to disable flags and
  now transfering it as a vector instead of bitlist.
* Fixed stupid == Bug.
* Added channel descriptions.
* Added params to disable single finger channels.
* Fixed the errornous attempt to fix the arg parameters :) they were correct
* Fixed Arg argument to Param
  Added Autostart Argument to standalone launchfile
* Set Default Simulation state to False
* Added Gui Argument to standalone launch file
* Fixed wrong param type
* Added Correct autostart, also changable for the hand now
* Added lauch params
* Removed subdirectory.
* Merge remote-tracking branch 'bergs_clone/master'
* Changed FM to be a shared pointer to not be a memory leak as it was created on the heap
  Added the Autostart functionality of the driver
* tweak materials
* remove unnecessary scales
* tweak materials, move out into materials.dae and add sed script to update materials in all .dae files.
  Usage: sed -i -f replace_effects.sed *.dae
* Introduced gui arg.
* remove excess material tags and add proper left hand support
* Merge branch 'master' of git://idsgit.fzi.de/ros/ros_s5fh
* new launch file for Automatica demo
* URDF fine tuning
* Rotated F31 by 180 deg.
* Merge git://idsgit.fzi.de/~nberg/ros/nbergs-ros_s5fh
* Deleted wrongly rotated STLs and added new versions of the DAEs.
  Dropped scaling factor.
* add launch file argument for namespace
* Merge branch 'master' of git://idsgit.fzi.de/~nberg/ros/nbergs-ros_s5fh
  Conflicts:
  s5fh_controller/launch/demo_with_websocket_server.launch
* change name of tf2_web_republisher node to what roslib.js expects
* Small fixes
* add single launch file including the necessary websocket nodes for the web UI
* add virtual middle finger "spread" link
* switch spread and tilt joins for pinky, index and ring finger, add models, unify indentation
  models aren't currently oriented correctly and have some wrong normals - this will be changed soon
* fix left hand (i.e. size == -1) ring finger and pinky movement
* fix orientation of pinky coordinate systems
* fix orientation of ring finger coordinate systems
* fix orientation of middle finger coordinate systems
* correct x axis for thumb and fix index finger
* fix orientation of thumb coordinate systems
* Better indentation for demo launch file + Parameter for USB Device
* Thread frequency adapted to fix consumer-producer-rate
* Fixed simulation toppics.
* Config file changes
* Added Thte testomat testnode. Its just a very very simple test node
  producing a sin wave for 2 fingers at default 50hz.
* Merge branch 'master' of idsgit.fzi.de:ros/ros_s5fh
* Small changes
* Fixed s5fh_controller stand-alone launch files.
* fixed launch file location
* removed s5fh_demo, moved content to s5fh_controller
* removed s5fh_gripper package, move content to s5fh_controller
* Updated Quick Commands,
  Removed RVIZ as default start
  Removed JointStatePublisher as default start
* Send target positions for all channels at once.
* Removed deprecated function.
* Merge branch 'master' of idsgit.fzi.de:ros/ros_s5fh
* Deleted useless file
* Merge branch 'master' of idsgit.fzi.de:ros/ros_s5fh
* removed hand_joint_control_out
* Overwrite dynamic reconfigure default value from launch file.
* Merge branch 'master' of idsgit.fzi.de:ros/ros_s5fh
* Added Quick commands for a fist :)
* Merge branch 'master' of idsgit.fzi.de:ros/ros_s5fh
* Topic changes to allow two interpolators.
* Silenced some annoying debug messages
* Removed warning message.
* Removed source list for joint state publisher.
* Merge branch 'master' of idsgit.fzi.de:ros/ros_s5fh
* Fixed toppic renaming.
  Fixed gui plugin toppic.
* Added node name to topics in rqt_reset_gui.
* Moved s5fh_controller launch file to correct folder.
* Removed dynamic reconfigure sliders for s5fh controller.
* Added s5fh urdf and s5fh_controller launch file for standalone usage of the five finger hand.
* cleaned a bit
* fiexed topic names in launch file
* Made node handle private.
* Merge branch 'master' of idsgit.fzi.de:ros/ros_s5fh
  Conflicts resolved:
  s5fh_controller/src/s5fh_controller.cpp
* Added channel feedback publisher.
  Publishes joint angles of each channel.
* moved lauch file to launch
* Merge branch 'master' of idsgit.fzi.de:ros/ros_s5fh
* added demo package, read joints from hand driver
* Added *.pyc files to git ignore.
* Fixed wrong out of bounds check for jointstate messages
* Added combined launch file for all hand arm components.
* Removed PYC file from version control.
* Added joint state subscriber reading positions and set target positions.
* Do not flush console.
* Adjusted config values to rad.
* Call finger manager disconnect function on program termination.
  With this we guarantee that all channels and controllers are deactivated.
* S5FH_Controller: Activated enableChannel after implementation in driver.
* S5FH_Controller: Disabled position feedback logging output to console.
* fixed name mapping
* get joints from interpolator/joint_commands
* attached hand to arm
* fixed joints in urdf
* implemented reset gui as rqt plugin
* removed reference to empty include dir
* Added test for position feedback.
* Added dynamic reconfigure sliders for controlling schunk hand.
* Removed sanity checks. This is now done in finger manager.
* Added info and error messages on reset.
* Fixed gripper CMakeLists.txt.
* Merge branch 'master' of idsgit.fzi.de:ros/ros_s5fh
* adapted urdf to schunk naming conventions
* Check bounds of input channel.
* Changed topic data to signed int.
* Added dynamic reconfigure to change the serial device parameter.
* moved to s5fh
* Fixed includes.
* s5fh_controller: Added topics and callback functions for connect, reset and enable.
* Merge branch 'master' of idsgit.fzi.de:ros/ros_s5fh
* Added ros node "s5fh_controller" for controlling the schunk five finger hand.
  This ros node uses driver_s5fh, icl_core and icl_comm libraries.
* added xacro s5fs file
* cleaned urdf
* removed model param from lauch file
* first complete version of s5fs_gripper with mimics
* first complete version of s5fs_gripper with mimics
* added CMakeLists.txt
* added first version of gripper urdf
* initial structure
* Contributors: Andreas Hermann, Georg Heppner, Lars Pfotzer, Nils Berg, Pascal Becker, Steffen Rühl, schunk2 user
