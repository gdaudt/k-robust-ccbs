# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Default target executed when no arguments are given to make.
default_target: all

.PHONY : default_target

# Allow only one "make -f Makefile2" at a time, but pass parallelism.
.NOTPARALLEL:


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
CMAKE_SOURCE_DIR = /home/gdaudt/Devel/k-robust-ccbs

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/gdaudt/Devel/k-robust-ccbs

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache

.PHONY : rebuild_cache/fast

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "No interactive CMake dialog available..."
	/usr/bin/cmake -E echo No\ interactive\ CMake\ dialog\ available.
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache

.PHONY : edit_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /home/gdaudt/Devel/k-robust-ccbs/CMakeFiles /home/gdaudt/Devel/k-robust-ccbs/CMakeFiles/progress.marks
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/gdaudt/Devel/k-robust-ccbs/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean

.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named k-robust-ccbs

# Build rule for target.
k-robust-ccbs: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 k-robust-ccbs
.PHONY : k-robust-ccbs

# fast build rule for target.
k-robust-ccbs/fast:
	$(MAKE) -f CMakeFiles/k-robust-ccbs.dir/build.make CMakeFiles/k-robust-ccbs.dir/build
.PHONY : k-robust-ccbs/fast

cbs.o: cbs.cpp.o

.PHONY : cbs.o

# target to build an object file
cbs.cpp.o:
	$(MAKE) -f CMakeFiles/k-robust-ccbs.dir/build.make CMakeFiles/k-robust-ccbs.dir/cbs.cpp.o
.PHONY : cbs.cpp.o

cbs.i: cbs.cpp.i

.PHONY : cbs.i

# target to preprocess a source file
cbs.cpp.i:
	$(MAKE) -f CMakeFiles/k-robust-ccbs.dir/build.make CMakeFiles/k-robust-ccbs.dir/cbs.cpp.i
.PHONY : cbs.cpp.i

cbs.s: cbs.cpp.s

.PHONY : cbs.s

# target to generate assembly for a file
cbs.cpp.s:
	$(MAKE) -f CMakeFiles/k-robust-ccbs.dir/build.make CMakeFiles/k-robust-ccbs.dir/cbs.cpp.s
.PHONY : cbs.cpp.s

config.o: config.cpp.o

.PHONY : config.o

# target to build an object file
config.cpp.o:
	$(MAKE) -f CMakeFiles/k-robust-ccbs.dir/build.make CMakeFiles/k-robust-ccbs.dir/config.cpp.o
.PHONY : config.cpp.o

config.i: config.cpp.i

.PHONY : config.i

# target to preprocess a source file
config.cpp.i:
	$(MAKE) -f CMakeFiles/k-robust-ccbs.dir/build.make CMakeFiles/k-robust-ccbs.dir/config.cpp.i
.PHONY : config.cpp.i

config.s: config.cpp.s

.PHONY : config.s

# target to generate assembly for a file
config.cpp.s:
	$(MAKE) -f CMakeFiles/k-robust-ccbs.dir/build.make CMakeFiles/k-robust-ccbs.dir/config.cpp.s
.PHONY : config.cpp.s

heuristic.o: heuristic.cpp.o

.PHONY : heuristic.o

# target to build an object file
heuristic.cpp.o:
	$(MAKE) -f CMakeFiles/k-robust-ccbs.dir/build.make CMakeFiles/k-robust-ccbs.dir/heuristic.cpp.o
.PHONY : heuristic.cpp.o

heuristic.i: heuristic.cpp.i

.PHONY : heuristic.i

# target to preprocess a source file
heuristic.cpp.i:
	$(MAKE) -f CMakeFiles/k-robust-ccbs.dir/build.make CMakeFiles/k-robust-ccbs.dir/heuristic.cpp.i
.PHONY : heuristic.cpp.i

heuristic.s: heuristic.cpp.s

.PHONY : heuristic.s

# target to generate assembly for a file
heuristic.cpp.s:
	$(MAKE) -f CMakeFiles/k-robust-ccbs.dir/build.make CMakeFiles/k-robust-ccbs.dir/heuristic.cpp.s
.PHONY : heuristic.cpp.s

main.o: main.cpp.o

.PHONY : main.o

# target to build an object file
main.cpp.o:
	$(MAKE) -f CMakeFiles/k-robust-ccbs.dir/build.make CMakeFiles/k-robust-ccbs.dir/main.cpp.o
.PHONY : main.cpp.o

main.i: main.cpp.i

.PHONY : main.i

# target to preprocess a source file
main.cpp.i:
	$(MAKE) -f CMakeFiles/k-robust-ccbs.dir/build.make CMakeFiles/k-robust-ccbs.dir/main.cpp.i
.PHONY : main.cpp.i

main.s: main.cpp.s

.PHONY : main.s

# target to generate assembly for a file
main.cpp.s:
	$(MAKE) -f CMakeFiles/k-robust-ccbs.dir/build.make CMakeFiles/k-robust-ccbs.dir/main.cpp.s
.PHONY : main.cpp.s

map.o: map.cpp.o

.PHONY : map.o

# target to build an object file
map.cpp.o:
	$(MAKE) -f CMakeFiles/k-robust-ccbs.dir/build.make CMakeFiles/k-robust-ccbs.dir/map.cpp.o
.PHONY : map.cpp.o

map.i: map.cpp.i

.PHONY : map.i

# target to preprocess a source file
map.cpp.i:
	$(MAKE) -f CMakeFiles/k-robust-ccbs.dir/build.make CMakeFiles/k-robust-ccbs.dir/map.cpp.i
.PHONY : map.cpp.i

map.s: map.cpp.s

.PHONY : map.s

# target to generate assembly for a file
map.cpp.s:
	$(MAKE) -f CMakeFiles/k-robust-ccbs.dir/build.make CMakeFiles/k-robust-ccbs.dir/map.cpp.s
.PHONY : map.cpp.s

sipp.o: sipp.cpp.o

.PHONY : sipp.o

# target to build an object file
sipp.cpp.o:
	$(MAKE) -f CMakeFiles/k-robust-ccbs.dir/build.make CMakeFiles/k-robust-ccbs.dir/sipp.cpp.o
.PHONY : sipp.cpp.o

sipp.i: sipp.cpp.i

.PHONY : sipp.i

# target to preprocess a source file
sipp.cpp.i:
	$(MAKE) -f CMakeFiles/k-robust-ccbs.dir/build.make CMakeFiles/k-robust-ccbs.dir/sipp.cpp.i
.PHONY : sipp.cpp.i

sipp.s: sipp.cpp.s

.PHONY : sipp.s

# target to generate assembly for a file
sipp.cpp.s:
	$(MAKE) -f CMakeFiles/k-robust-ccbs.dir/build.make CMakeFiles/k-robust-ccbs.dir/sipp.cpp.s
.PHONY : sipp.cpp.s

task.o: task.cpp.o

.PHONY : task.o

# target to build an object file
task.cpp.o:
	$(MAKE) -f CMakeFiles/k-robust-ccbs.dir/build.make CMakeFiles/k-robust-ccbs.dir/task.cpp.o
.PHONY : task.cpp.o

task.i: task.cpp.i

.PHONY : task.i

# target to preprocess a source file
task.cpp.i:
	$(MAKE) -f CMakeFiles/k-robust-ccbs.dir/build.make CMakeFiles/k-robust-ccbs.dir/task.cpp.i
.PHONY : task.cpp.i

task.s: task.cpp.s

.PHONY : task.s

# target to generate assembly for a file
task.cpp.s:
	$(MAKE) -f CMakeFiles/k-robust-ccbs.dir/build.make CMakeFiles/k-robust-ccbs.dir/task.cpp.s
.PHONY : task.cpp.s

tinyxml2.o: tinyxml2.cpp.o

.PHONY : tinyxml2.o

# target to build an object file
tinyxml2.cpp.o:
	$(MAKE) -f CMakeFiles/k-robust-ccbs.dir/build.make CMakeFiles/k-robust-ccbs.dir/tinyxml2.cpp.o
.PHONY : tinyxml2.cpp.o

tinyxml2.i: tinyxml2.cpp.i

.PHONY : tinyxml2.i

# target to preprocess a source file
tinyxml2.cpp.i:
	$(MAKE) -f CMakeFiles/k-robust-ccbs.dir/build.make CMakeFiles/k-robust-ccbs.dir/tinyxml2.cpp.i
.PHONY : tinyxml2.cpp.i

tinyxml2.s: tinyxml2.cpp.s

.PHONY : tinyxml2.s

# target to generate assembly for a file
tinyxml2.cpp.s:
	$(MAKE) -f CMakeFiles/k-robust-ccbs.dir/build.make CMakeFiles/k-robust-ccbs.dir/tinyxml2.cpp.s
.PHONY : tinyxml2.cpp.s

xml_logger.o: xml_logger.cpp.o

.PHONY : xml_logger.o

# target to build an object file
xml_logger.cpp.o:
	$(MAKE) -f CMakeFiles/k-robust-ccbs.dir/build.make CMakeFiles/k-robust-ccbs.dir/xml_logger.cpp.o
.PHONY : xml_logger.cpp.o

xml_logger.i: xml_logger.cpp.i

.PHONY : xml_logger.i

# target to preprocess a source file
xml_logger.cpp.i:
	$(MAKE) -f CMakeFiles/k-robust-ccbs.dir/build.make CMakeFiles/k-robust-ccbs.dir/xml_logger.cpp.i
.PHONY : xml_logger.cpp.i

xml_logger.s: xml_logger.cpp.s

.PHONY : xml_logger.s

# target to generate assembly for a file
xml_logger.cpp.s:
	$(MAKE) -f CMakeFiles/k-robust-ccbs.dir/build.make CMakeFiles/k-robust-ccbs.dir/xml_logger.cpp.s
.PHONY : xml_logger.cpp.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... rebuild_cache"
	@echo "... edit_cache"
	@echo "... k-robust-ccbs"
	@echo "... cbs.o"
	@echo "... cbs.i"
	@echo "... cbs.s"
	@echo "... config.o"
	@echo "... config.i"
	@echo "... config.s"
	@echo "... heuristic.o"
	@echo "... heuristic.i"
	@echo "... heuristic.s"
	@echo "... main.o"
	@echo "... main.i"
	@echo "... main.s"
	@echo "... map.o"
	@echo "... map.i"
	@echo "... map.s"
	@echo "... sipp.o"
	@echo "... sipp.i"
	@echo "... sipp.s"
	@echo "... task.o"
	@echo "... task.i"
	@echo "... task.s"
	@echo "... tinyxml2.o"
	@echo "... tinyxml2.i"
	@echo "... tinyxml2.s"
	@echo "... xml_logger.o"
	@echo "... xml_logger.i"
	@echo "... xml_logger.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

