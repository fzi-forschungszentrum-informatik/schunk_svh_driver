cmake_minimum_required(VERSION 2.8.3)
project(s5fs_gripper)
find_package(catkin REQUIRED)

catkin_package()

# mesh file generations
# iterate through all the stl files to:
#  1. convert to *.iv files
#  2. generate convex decomposed .stl


foreach(dir meshes urdf)
   install(DIRECTORY ${dir}/ 
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)
