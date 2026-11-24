# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/tianyu-tony-zhou/radar4motion-ros2/build/radar4motion/_deps/tessil-src"
  "/home/tianyu-tony-zhou/radar4motion-ros2/build/radar4motion/_deps/tessil-build"
  "/home/tianyu-tony-zhou/radar4motion-ros2/build/radar4motion/_deps/tessil-subbuild/tessil-populate-prefix"
  "/home/tianyu-tony-zhou/radar4motion-ros2/build/radar4motion/_deps/tessil-subbuild/tessil-populate-prefix/tmp"
  "/home/tianyu-tony-zhou/radar4motion-ros2/build/radar4motion/_deps/tessil-subbuild/tessil-populate-prefix/src/tessil-populate-stamp"
  "/home/tianyu-tony-zhou/radar4motion-ros2/build/radar4motion/_deps/tessil-subbuild/tessil-populate-prefix/src"
  "/home/tianyu-tony-zhou/radar4motion-ros2/build/radar4motion/_deps/tessil-subbuild/tessil-populate-prefix/src/tessil-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/tianyu-tony-zhou/radar4motion-ros2/build/radar4motion/_deps/tessil-subbuild/tessil-populate-prefix/src/tessil-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/tianyu-tony-zhou/radar4motion-ros2/build/radar4motion/_deps/tessil-subbuild/tessil-populate-prefix/src/tessil-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
