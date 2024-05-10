# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/tomoyaueno/esp/esp-idf-v5.2.1/components/bootloader/subproject"
  "/Users/tomoyaueno/jaist/1-1/iot/proj/assignment/indivisual-a/bmp-180/build/bootloader"
  "/Users/tomoyaueno/jaist/1-1/iot/proj/assignment/indivisual-a/bmp-180/build/bootloader-prefix"
  "/Users/tomoyaueno/jaist/1-1/iot/proj/assignment/indivisual-a/bmp-180/build/bootloader-prefix/tmp"
  "/Users/tomoyaueno/jaist/1-1/iot/proj/assignment/indivisual-a/bmp-180/build/bootloader-prefix/src/bootloader-stamp"
  "/Users/tomoyaueno/jaist/1-1/iot/proj/assignment/indivisual-a/bmp-180/build/bootloader-prefix/src"
  "/Users/tomoyaueno/jaist/1-1/iot/proj/assignment/indivisual-a/bmp-180/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/tomoyaueno/jaist/1-1/iot/proj/assignment/indivisual-a/bmp-180/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/tomoyaueno/jaist/1-1/iot/proj/assignment/indivisual-a/bmp-180/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
