# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Produce verbose output by default.
VERBOSE = 1

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
CMAKE_SOURCE_DIR = /home/waldezjr/icub/software/src/qpOASES

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/waldezjr/icub/software/src/qpOASES/build

# Include any dependencies generated for this target.
include CMakeFiles/example1a.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/example1a.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/example1a.dir/flags.make

CMakeFiles/example1a.dir/examples/example1a.cpp.o: CMakeFiles/example1a.dir/flags.make
CMakeFiles/example1a.dir/examples/example1a.cpp.o: ../examples/example1a.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/waldezjr/icub/software/src/qpOASES/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/example1a.dir/examples/example1a.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/example1a.dir/examples/example1a.cpp.o -c /home/waldezjr/icub/software/src/qpOASES/examples/example1a.cpp

CMakeFiles/example1a.dir/examples/example1a.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/example1a.dir/examples/example1a.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/waldezjr/icub/software/src/qpOASES/examples/example1a.cpp > CMakeFiles/example1a.dir/examples/example1a.cpp.i

CMakeFiles/example1a.dir/examples/example1a.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/example1a.dir/examples/example1a.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/waldezjr/icub/software/src/qpOASES/examples/example1a.cpp -o CMakeFiles/example1a.dir/examples/example1a.cpp.s

CMakeFiles/example1a.dir/examples/example1a.cpp.o.requires:

.PHONY : CMakeFiles/example1a.dir/examples/example1a.cpp.o.requires

CMakeFiles/example1a.dir/examples/example1a.cpp.o.provides: CMakeFiles/example1a.dir/examples/example1a.cpp.o.requires
	$(MAKE) -f CMakeFiles/example1a.dir/build.make CMakeFiles/example1a.dir/examples/example1a.cpp.o.provides.build
.PHONY : CMakeFiles/example1a.dir/examples/example1a.cpp.o.provides

CMakeFiles/example1a.dir/examples/example1a.cpp.o.provides.build: CMakeFiles/example1a.dir/examples/example1a.cpp.o


# Object files for target example1a
example1a_OBJECTS = \
"CMakeFiles/example1a.dir/examples/example1a.cpp.o"

# External object files for target example1a
example1a_EXTERNAL_OBJECTS =

bin/example1a: CMakeFiles/example1a.dir/examples/example1a.cpp.o
bin/example1a: CMakeFiles/example1a.dir/build.make
bin/example1a: libs/libqpOASES.a
bin/example1a: CMakeFiles/example1a.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/waldezjr/icub/software/src/qpOASES/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable bin/example1a"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/example1a.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/example1a.dir/build: bin/example1a

.PHONY : CMakeFiles/example1a.dir/build

CMakeFiles/example1a.dir/requires: CMakeFiles/example1a.dir/examples/example1a.cpp.o.requires

.PHONY : CMakeFiles/example1a.dir/requires

CMakeFiles/example1a.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/example1a.dir/cmake_clean.cmake
.PHONY : CMakeFiles/example1a.dir/clean

CMakeFiles/example1a.dir/depend:
	cd /home/waldezjr/icub/software/src/qpOASES/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/waldezjr/icub/software/src/qpOASES /home/waldezjr/icub/software/src/qpOASES /home/waldezjr/icub/software/src/qpOASES/build /home/waldezjr/icub/software/src/qpOASES/build /home/waldezjr/icub/software/src/qpOASES/build/CMakeFiles/example1a.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/example1a.dir/depend

