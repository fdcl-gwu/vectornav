# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.10.2/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.10.2/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/kanishke/Downloads/vnproglib-1.1/c/examples/getting_started

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/kanishke/Downloads/vnproglib-1.1/c/examples/getting_started/build

# Include any dependencies generated for this target.
include CMakeFiles/getting_started.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/getting_started.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/getting_started.dir/flags.make

CMakeFiles/getting_started.dir/main.c.o: CMakeFiles/getting_started.dir/flags.make
CMakeFiles/getting_started.dir/main.c.o: ../main.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/kanishke/Downloads/vnproglib-1.1/c/examples/getting_started/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/getting_started.dir/main.c.o"
	/Library/Developer/CommandLineTools/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/getting_started.dir/main.c.o   -c /Users/kanishke/Downloads/vnproglib-1.1/c/examples/getting_started/main.c

CMakeFiles/getting_started.dir/main.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/getting_started.dir/main.c.i"
	/Library/Developer/CommandLineTools/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/kanishke/Downloads/vnproglib-1.1/c/examples/getting_started/main.c > CMakeFiles/getting_started.dir/main.c.i

CMakeFiles/getting_started.dir/main.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/getting_started.dir/main.c.s"
	/Library/Developer/CommandLineTools/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/kanishke/Downloads/vnproglib-1.1/c/examples/getting_started/main.c -o CMakeFiles/getting_started.dir/main.c.s

CMakeFiles/getting_started.dir/main.c.o.requires:

.PHONY : CMakeFiles/getting_started.dir/main.c.o.requires

CMakeFiles/getting_started.dir/main.c.o.provides: CMakeFiles/getting_started.dir/main.c.o.requires
	$(MAKE) -f CMakeFiles/getting_started.dir/build.make CMakeFiles/getting_started.dir/main.c.o.provides.build
.PHONY : CMakeFiles/getting_started.dir/main.c.o.provides

CMakeFiles/getting_started.dir/main.c.o.provides.build: CMakeFiles/getting_started.dir/main.c.o


# Object files for target getting_started
getting_started_OBJECTS = \
"CMakeFiles/getting_started.dir/main.c.o"

# External object files for target getting_started
getting_started_EXTERNAL_OBJECTS =

getting_started: CMakeFiles/getting_started.dir/main.c.o
getting_started: CMakeFiles/getting_started.dir/build.make
getting_started: libvnc/liblibvnc.a
getting_started: CMakeFiles/getting_started.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/kanishke/Downloads/vnproglib-1.1/c/examples/getting_started/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C executable getting_started"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/getting_started.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/getting_started.dir/build: getting_started

.PHONY : CMakeFiles/getting_started.dir/build

CMakeFiles/getting_started.dir/requires: CMakeFiles/getting_started.dir/main.c.o.requires

.PHONY : CMakeFiles/getting_started.dir/requires

CMakeFiles/getting_started.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/getting_started.dir/cmake_clean.cmake
.PHONY : CMakeFiles/getting_started.dir/clean

CMakeFiles/getting_started.dir/depend:
	cd /Users/kanishke/Downloads/vnproglib-1.1/c/examples/getting_started/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/kanishke/Downloads/vnproglib-1.1/c/examples/getting_started /Users/kanishke/Downloads/vnproglib-1.1/c/examples/getting_started /Users/kanishke/Downloads/vnproglib-1.1/c/examples/getting_started/build /Users/kanishke/Downloads/vnproglib-1.1/c/examples/getting_started/build /Users/kanishke/Downloads/vnproglib-1.1/c/examples/getting_started/build/CMakeFiles/getting_started.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/getting_started.dir/depend
