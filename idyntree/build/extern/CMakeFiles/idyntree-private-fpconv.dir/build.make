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
CMAKE_SOURCE_DIR = /home/jcacace/quad_ws/src/idyntree

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jcacace/quad_ws/src/idyntree/build

# Include any dependencies generated for this target.
include extern/CMakeFiles/idyntree-private-fpconv.dir/depend.make

# Include the progress variables for this target.
include extern/CMakeFiles/idyntree-private-fpconv.dir/progress.make

# Include the compile flags for this target's objects.
include extern/CMakeFiles/idyntree-private-fpconv.dir/flags.make

extern/CMakeFiles/idyntree-private-fpconv.dir/fpconv/fpconv.c.o: extern/CMakeFiles/idyntree-private-fpconv.dir/flags.make
extern/CMakeFiles/idyntree-private-fpconv.dir/fpconv/fpconv.c.o: ../extern/fpconv/fpconv.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jcacace/quad_ws/src/idyntree/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object extern/CMakeFiles/idyntree-private-fpconv.dir/fpconv/fpconv.c.o"
	cd /home/jcacace/quad_ws/src/idyntree/build/extern && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/idyntree-private-fpconv.dir/fpconv/fpconv.c.o   -c /home/jcacace/quad_ws/src/idyntree/extern/fpconv/fpconv.c

extern/CMakeFiles/idyntree-private-fpconv.dir/fpconv/fpconv.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/idyntree-private-fpconv.dir/fpconv/fpconv.c.i"
	cd /home/jcacace/quad_ws/src/idyntree/build/extern && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/jcacace/quad_ws/src/idyntree/extern/fpconv/fpconv.c > CMakeFiles/idyntree-private-fpconv.dir/fpconv/fpconv.c.i

extern/CMakeFiles/idyntree-private-fpconv.dir/fpconv/fpconv.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/idyntree-private-fpconv.dir/fpconv/fpconv.c.s"
	cd /home/jcacace/quad_ws/src/idyntree/build/extern && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/jcacace/quad_ws/src/idyntree/extern/fpconv/fpconv.c -o CMakeFiles/idyntree-private-fpconv.dir/fpconv/fpconv.c.s

idyntree-private-fpconv: extern/CMakeFiles/idyntree-private-fpconv.dir/fpconv/fpconv.c.o
idyntree-private-fpconv: extern/CMakeFiles/idyntree-private-fpconv.dir/build.make

.PHONY : idyntree-private-fpconv

# Rule to build all files generated by this target.
extern/CMakeFiles/idyntree-private-fpconv.dir/build: idyntree-private-fpconv

.PHONY : extern/CMakeFiles/idyntree-private-fpconv.dir/build

extern/CMakeFiles/idyntree-private-fpconv.dir/clean:
	cd /home/jcacace/quad_ws/src/idyntree/build/extern && $(CMAKE_COMMAND) -P CMakeFiles/idyntree-private-fpconv.dir/cmake_clean.cmake
.PHONY : extern/CMakeFiles/idyntree-private-fpconv.dir/clean

extern/CMakeFiles/idyntree-private-fpconv.dir/depend:
	cd /home/jcacace/quad_ws/src/idyntree/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jcacace/quad_ws/src/idyntree /home/jcacace/quad_ws/src/idyntree/extern /home/jcacace/quad_ws/src/idyntree/build /home/jcacace/quad_ws/src/idyntree/build/extern /home/jcacace/quad_ws/src/idyntree/build/extern/CMakeFiles/idyntree-private-fpconv.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : extern/CMakeFiles/idyntree-private-fpconv.dir/depend

