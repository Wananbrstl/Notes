# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
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
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/lbn/slam/use_Sophus

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lbn/slam/use_Sophus/build

# Include any dependencies generated for this target.
include CMakeFiles/test_agin.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/test_agin.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test_agin.dir/flags.make

CMakeFiles/test_agin.dir/src/test_Souphus_agin.cpp.o: CMakeFiles/test_agin.dir/flags.make
CMakeFiles/test_agin.dir/src/test_Souphus_agin.cpp.o: ../src/test_Souphus_agin.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lbn/slam/use_Sophus/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/test_agin.dir/src/test_Souphus_agin.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_agin.dir/src/test_Souphus_agin.cpp.o -c /home/lbn/slam/use_Sophus/src/test_Souphus_agin.cpp

CMakeFiles/test_agin.dir/src/test_Souphus_agin.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_agin.dir/src/test_Souphus_agin.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lbn/slam/use_Sophus/src/test_Souphus_agin.cpp > CMakeFiles/test_agin.dir/src/test_Souphus_agin.cpp.i

CMakeFiles/test_agin.dir/src/test_Souphus_agin.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_agin.dir/src/test_Souphus_agin.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lbn/slam/use_Sophus/src/test_Souphus_agin.cpp -o CMakeFiles/test_agin.dir/src/test_Souphus_agin.cpp.s

# Object files for target test_agin
test_agin_OBJECTS = \
"CMakeFiles/test_agin.dir/src/test_Souphus_agin.cpp.o"

# External object files for target test_agin
test_agin_EXTERNAL_OBJECTS =

test_agin: CMakeFiles/test_agin.dir/src/test_Souphus_agin.cpp.o
test_agin: CMakeFiles/test_agin.dir/build.make
test_agin: /home/lbn/slam/Sophus/build/libSophus.so
test_agin: CMakeFiles/test_agin.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lbn/slam/use_Sophus/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable test_agin"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_agin.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test_agin.dir/build: test_agin

.PHONY : CMakeFiles/test_agin.dir/build

CMakeFiles/test_agin.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test_agin.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test_agin.dir/clean

CMakeFiles/test_agin.dir/depend:
	cd /home/lbn/slam/use_Sophus/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lbn/slam/use_Sophus /home/lbn/slam/use_Sophus /home/lbn/slam/use_Sophus/build /home/lbn/slam/use_Sophus/build /home/lbn/slam/use_Sophus/build/CMakeFiles/test_agin.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test_agin.dir/depend

