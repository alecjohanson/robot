cmake_minimum_required (VERSION 2.6 FATAL_ERROR)
include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init ()

set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rosbuild_add_executable (rcg src/recognition.cpp)

include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
pkg_check_modules(EIGEN REQUIRED eigen3)
include_directories(${EIGEN_INCLUDE_DIRS})
link_directories(${EIGEN_LIBRARY_DIRS})
target_link_libraries(rcg ${EIGEN_LIBRARIES})

pkg_check_modules(PCLKP REQUIRED pcl_keypoints-1.7)
include_directories(${PCLKP_INCLUDE_DIRS})
link_directories(${PCLKP_LIBRARY_DIRS})
target_link_libraries(rcg ${PCLKP_LIBRARIES})

pkg_check_modules(PCLREC REQUIRED pcl_recognition-1.7)
include_directories(${PCLREC_INCLUDE_DIRS})
link_directories(${PCLREC_LIBRARY_DIRS})
target_link_libraries(rcg ${PCLREC_LIBRARIES})

pkg_check_modules(PCLVL REQUIRED pcl_visualization-1.7)
include_directories(${PCLVL_INCLUDE_DIRS})
link_directories(${PCLVL_LIBRARY_DIRS})
target_link_libraries(rcg ${PCLVL_LIBRARIES})


find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})
link_directories(${VTK_LIBRARY_DIRS})


rosbuild_add_boost_directories()
rosbuild_link_boost(rcg system filesystem)
#target_link_libraries(rcg /usr/lib/libvtkCommon.so.5.8 /usr/lib/libvtkFiltering.so.5.8 /usr/lib/libboost_filesystem.so.1.46.1 pcl_common)
