# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/kaisar/ws_livox/src/Livox-SDK2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kaisar/ws_livox/src/Livox-SDK2/build

# Include any dependencies generated for this target.
include samples/multi_lidars_upgrade/CMakeFiles/multi_lidars_upgrade.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include samples/multi_lidars_upgrade/CMakeFiles/multi_lidars_upgrade.dir/compiler_depend.make

# Include the progress variables for this target.
include samples/multi_lidars_upgrade/CMakeFiles/multi_lidars_upgrade.dir/progress.make

# Include the compile flags for this target's objects.
include samples/multi_lidars_upgrade/CMakeFiles/multi_lidars_upgrade.dir/flags.make

samples/multi_lidars_upgrade/CMakeFiles/multi_lidars_upgrade.dir/main.cpp.o: samples/multi_lidars_upgrade/CMakeFiles/multi_lidars_upgrade.dir/flags.make
samples/multi_lidars_upgrade/CMakeFiles/multi_lidars_upgrade.dir/main.cpp.o: ../samples/multi_lidars_upgrade/main.cpp
samples/multi_lidars_upgrade/CMakeFiles/multi_lidars_upgrade.dir/main.cpp.o: samples/multi_lidars_upgrade/CMakeFiles/multi_lidars_upgrade.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kaisar/ws_livox/src/Livox-SDK2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object samples/multi_lidars_upgrade/CMakeFiles/multi_lidars_upgrade.dir/main.cpp.o"
	cd /home/kaisar/ws_livox/src/Livox-SDK2/build/samples/multi_lidars_upgrade && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT samples/multi_lidars_upgrade/CMakeFiles/multi_lidars_upgrade.dir/main.cpp.o -MF CMakeFiles/multi_lidars_upgrade.dir/main.cpp.o.d -o CMakeFiles/multi_lidars_upgrade.dir/main.cpp.o -c /home/kaisar/ws_livox/src/Livox-SDK2/samples/multi_lidars_upgrade/main.cpp

samples/multi_lidars_upgrade/CMakeFiles/multi_lidars_upgrade.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/multi_lidars_upgrade.dir/main.cpp.i"
	cd /home/kaisar/ws_livox/src/Livox-SDK2/build/samples/multi_lidars_upgrade && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kaisar/ws_livox/src/Livox-SDK2/samples/multi_lidars_upgrade/main.cpp > CMakeFiles/multi_lidars_upgrade.dir/main.cpp.i

samples/multi_lidars_upgrade/CMakeFiles/multi_lidars_upgrade.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/multi_lidars_upgrade.dir/main.cpp.s"
	cd /home/kaisar/ws_livox/src/Livox-SDK2/build/samples/multi_lidars_upgrade && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kaisar/ws_livox/src/Livox-SDK2/samples/multi_lidars_upgrade/main.cpp -o CMakeFiles/multi_lidars_upgrade.dir/main.cpp.s

# Object files for target multi_lidars_upgrade
multi_lidars_upgrade_OBJECTS = \
"CMakeFiles/multi_lidars_upgrade.dir/main.cpp.o"

# External object files for target multi_lidars_upgrade
multi_lidars_upgrade_EXTERNAL_OBJECTS =

samples/multi_lidars_upgrade/multi_lidars_upgrade: samples/multi_lidars_upgrade/CMakeFiles/multi_lidars_upgrade.dir/main.cpp.o
samples/multi_lidars_upgrade/multi_lidars_upgrade: samples/multi_lidars_upgrade/CMakeFiles/multi_lidars_upgrade.dir/build.make
samples/multi_lidars_upgrade/multi_lidars_upgrade: sdk_core/liblivox_lidar_sdk_static.a
samples/multi_lidars_upgrade/multi_lidars_upgrade: samples/multi_lidars_upgrade/CMakeFiles/multi_lidars_upgrade.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kaisar/ws_livox/src/Livox-SDK2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable multi_lidars_upgrade"
	cd /home/kaisar/ws_livox/src/Livox-SDK2/build/samples/multi_lidars_upgrade && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/multi_lidars_upgrade.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
samples/multi_lidars_upgrade/CMakeFiles/multi_lidars_upgrade.dir/build: samples/multi_lidars_upgrade/multi_lidars_upgrade
.PHONY : samples/multi_lidars_upgrade/CMakeFiles/multi_lidars_upgrade.dir/build

samples/multi_lidars_upgrade/CMakeFiles/multi_lidars_upgrade.dir/clean:
	cd /home/kaisar/ws_livox/src/Livox-SDK2/build/samples/multi_lidars_upgrade && $(CMAKE_COMMAND) -P CMakeFiles/multi_lidars_upgrade.dir/cmake_clean.cmake
.PHONY : samples/multi_lidars_upgrade/CMakeFiles/multi_lidars_upgrade.dir/clean

samples/multi_lidars_upgrade/CMakeFiles/multi_lidars_upgrade.dir/depend:
	cd /home/kaisar/ws_livox/src/Livox-SDK2/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kaisar/ws_livox/src/Livox-SDK2 /home/kaisar/ws_livox/src/Livox-SDK2/samples/multi_lidars_upgrade /home/kaisar/ws_livox/src/Livox-SDK2/build /home/kaisar/ws_livox/src/Livox-SDK2/build/samples/multi_lidars_upgrade /home/kaisar/ws_livox/src/Livox-SDK2/build/samples/multi_lidars_upgrade/CMakeFiles/multi_lidars_upgrade.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : samples/multi_lidars_upgrade/CMakeFiles/multi_lidars_upgrade.dir/depend
