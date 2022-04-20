# SPDX-License-Identifier: Apache-2.0

######################################
# The use of this file is deprecated #
######################################

# To build a Zephyr application it must start with one of those lines:
#
# find_package(Zephyr)
# find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
#
# The `REQUIRED HINTS $ENV{ZEPHYR_BASE}` variant is required for any application
# inside the Zephyr repository.
#
# It exists to reduce boilerplate code that Zephyr expects to be in
# application CMakeLists.txt code.

# CMake version 3.20 is the real minimum supported version.
#
# Unfortunately CMake requires the toplevel CMakeLists.txt file to
# define the required version, not even invoking it from an included
# file, like boilerplate.cmake, is sufficient. It is however permitted
# to have multiple invocations of cmake_minimum_required.
#
# Under these restraints we use a second 'cmake_minimum_required'
# invocation in every toplevel CMakeLists.txt.
cmake_minimum_required(VERSION 3.20.0)

define_property(GLOBAL PROPERTY ZEPHYR_LIBS
    BRIEF_DOCS "Global list of all Zephyr CMake libs that should be linked in"
    FULL_DOCS  "Global list of all Zephyr CMake libs that should be linked in.
zephyr_library() appends libs to this list.")
set_property(GLOBAL PROPERTY ZEPHYR_LIBS "")

define_property(GLOBAL PROPERTY ZEPHYR_INTERFACE_LIBS
    BRIEF_DOCS "Global list of all Zephyr interface libs that should be linked in."
    FULL_DOCS  "Global list of all Zephyr interface libs that should be linked in.
zephyr_interface_library_named() appends libs to this list.")
set_property(GLOBAL PROPERTY ZEPHYR_INTERFACE_LIBS "")

define_property(GLOBAL PROPERTY GENERATED_APP_SOURCE_FILES
  BRIEF_DOCS "Source files that are generated after Zephyr has been linked once."
  FULL_DOCS "\
Source files that are generated after Zephyr has been linked once.\
May include dev_handles.c etc."
  )
set_property(GLOBAL PROPERTY GENERATED_APP_SOURCE_FILES "")

if(NOT DEFINED Zephyr_DIR)
  # When `find_package(Zephyr)` is used then `Zephyr_DIR` is defined, else
  # old style inclusion is used. Warning is only printed in first invocation.
  message(WARNING "Loading of Zephyr boilerplate.cmake directly is deprecated, "
          "please use 'find_package(Zephyr)'"
  )
endif()

find_package(Zephyr REQUIRED PATHS ${CMAKE_CURRENT_LIST_DIR}/../.. NO_DEFAULT_PATH)
