cmake_minimum_required(VERSION 2.8.3)
project(s5fh_gripper)
find_package(catkin REQUIRED)
find_package(xacro REQUIRED)

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

set(s5sh_files "")


# Dependency files
file(GLOB model_xacro_files ${CMAKE_CURRENT_SOURCE_DIR}/description/urdf/*.xacro) 

foreach(it ${model_xacro_files})
  get_filename_component(basepath ${it} PATH)
  get_filename_component(basename ${it} NAME_WE)
  
  message(" processing world file :",${basepath}," : ",${basename})
  
  set(expanded_file "${basepath}/${basename}.world")
  message("    produce expanded file :",${expanded_file})
  
  xacro_add_xacro_file(${it} ${expanded_file})
  
  set(world_files ${world_files} ${expanded_file})
endforeach(it)

add_custom_target(media_files ALL DEPENDS ${world_files})