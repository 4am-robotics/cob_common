cmake_minimum_required(VERSION 2.8.3)
project(desire_description)

find_package(catkin REQUIRED COMPONENTS convex_decomposition ivcon)

# iterate through all the stl files to:
#  1. convert to *.iv files
#  2. generate convex decomposed .stl
file(GLOB desire_stl_files ${CMAKE_CURRENT_SOURCE_DIR}/ros/meshes/*/*.stl)
set(desire_gen_files "")

foreach(it ${desire_stl_files})
  get_filename_component(basepath ${it} PATH)
  get_filename_component(basename ${it} NAME_WE)

  IF ( ${basename} MATCHES "_convex" )

    message("ignoring stale .._convex.stl file:",${basename})

  ELSE ( ${basename} MATCHES "_convex" )

    # create subdirectory convex
    add_custom_command(
      OUTPUT ${basepath}/convex
      # CMake 2.4 doesn't offer the make_directory command.
      #COMMAND ${CMAKE_COMMAND} -E make_directory
      COMMAND mkdir -p
      ARGS ${basepath}/convex)

    # create subdirectory iv
    add_custom_command(
      OUTPUT ${basepath}/iv
      # CMake 2.4 doesn't offer the make_directory command.
      #COMMAND ${CMAKE_COMMAND} -E make_directory
      COMMAND mkdir -p
      ARGS ${basepath}/iv)

    #create obj files for convex decomposition from stl files
    add_custom_command(
      OUTPUT ${basepath}/convex/${basename}.obj
      COMMAND ivcon
      ARGS ${it} ${basepath}/convex/${basename}.obj
      DEPENDS ${it} ${basepath}/convex)

    set(desire_gen_files ${desire_gen_files} ${basepath}/convex/${basename}.obj)

    #convex decompose object files
    add_custom_command(
      OUTPUT ${basepath}/convex/${basename}_convex.obj
      COMMAND convex_decomposition
      ARGS ${basepath}/convex/${basename}.obj -v12 -p10
      DEPENDS ${basepath}/convex/${basename}.obj ${basepath}/convex)

    set(desire_gen_files ${desire_gen_files} ${basepath}/convex/${basename}_convex.obj)

    #convert obj files back to stlb, put in directory named convex
    add_custom_command(
      OUTPUT ${basepath}/convex/${basename}_convex.stlb
      COMMAND ivcon
      ARGS ${basepath}/convex/${basename}_convex.obj ${basepath}/convex/${basename}_convex.stlb
      DEPENDS ${it} ${basepath}/convex ${basepath}/convex/${basename}_convex.obj)

    set(desire_gen_files ${desire_gen_files} ${basepath}/convex/${basename}_convex.stlb)

    #convert obj files back to stla, put in directory named convex
    add_custom_command(
      OUTPUT ${basepath}/convex/${basename}_convex.stla
      COMMAND ivcon
      ARGS ${basepath}/convex/${basename}_convex.obj ${basepath}/convex/${basename}_convex.stla
      DEPENDS ${it} ${basepath}/convex ${basepath}/convex/${basename}_convex.obj)

    set(desire_gen_files ${desire_gen_files} ${basepath}/convex/${basename}_convex.stla)

    #create iv files
    add_custom_command(
      OUTPUT ${basepath}/iv/${basename}.iv
      COMMAND ivcon
      ARGS ${it} ${basepath}/iv/${basename}.iv
      DEPENDS ${basepath}/iv ${it})
    add_custom_command(
      OUTPUT ${basepath}/convex/${basename}_convex.iv
      COMMAND ivcon
      ARGS ${basepath}/convex/${basename}_convex.obj ${basepath}/convex/${basename}_convex.iv
      DEPENDS ${it} ${basepath}/convex ${basepath}/convex/${basename}_convex.obj)
     
    set(desire_gen_files ${desire_gen_files} ${basepath}/iv/${basename}.iv ${basepath}/convex/${basename}_convex.iv)


  ENDIF ( ${basename} MATCHES "_convex" )

endforeach(it)

add_custom_target(media_files_desire ALL DEPENDS ${desire_gen_files})

#catkin_package(CATKIN_DEPENDS xacro)
catkin_package(CATKIN_DEPENDS)

#catkin_add_gtest(test_urdf_desire test/test_urdf.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})

foreach(dir meshes gazebo materials robots documents urdf)
   install(DIRECTORY ${dir}/ 
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)


