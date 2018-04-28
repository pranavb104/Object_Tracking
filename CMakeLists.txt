cmake_minimum_required( VERSION 2.8 )
set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH} )

project( rs_run )
add_executable( rs_run rs_run.cpp )
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "rs_run" )

# Find Packages
find_package( PCL 1.8 REQUIRED )


if( PCL_FOUND )
  # Additional Include Directories
  include_directories( ${PCL_INCLUDE_DIRS} )

  # Preprocessor Definitions
  add_definitions( ${PCL_DEFINITIONS} )

  # Additional Library Directories
  link_directories( ${PCL_LIBRARY_DIRS} )

  # Additional Dependencies
  target_link_libraries( rs_run ${PCL_LIBRARIES} )
endif()
