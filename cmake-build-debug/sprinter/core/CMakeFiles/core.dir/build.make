# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

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
CMAKE_COMMAND = /home/zahorack/.local/clion/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/zahorack/.local/clion/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/zahorack/projects/c_projects_stm/sprinter

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zahorack/projects/c_projects_stm/sprinter/cmake-build-debug

# Include any dependencies generated for this target.
include sprinter/core/CMakeFiles/core.dir/depend.make

# Include the progress variables for this target.
include sprinter/core/CMakeFiles/core.dir/progress.make

# Include the compile flags for this target's objects.
include sprinter/core/CMakeFiles/core.dir/flags.make

sprinter/core/CMakeFiles/core.dir/src/log.cpp.obj: sprinter/core/CMakeFiles/core.dir/flags.make
sprinter/core/CMakeFiles/core.dir/src/log.cpp.obj: ../sprinter/core/src/log.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zahorack/projects/c_projects_stm/sprinter/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object sprinter/core/CMakeFiles/core.dir/src/log.cpp.obj"
	cd /home/zahorack/projects/c_projects_stm/sprinter/cmake-build-debug/sprinter/core && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/core.dir/src/log.cpp.obj -c /home/zahorack/projects/c_projects_stm/sprinter/sprinter/core/src/log.cpp

sprinter/core/CMakeFiles/core.dir/src/log.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/core.dir/src/log.cpp.i"
	cd /home/zahorack/projects/c_projects_stm/sprinter/cmake-build-debug/sprinter/core && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zahorack/projects/c_projects_stm/sprinter/sprinter/core/src/log.cpp > CMakeFiles/core.dir/src/log.cpp.i

sprinter/core/CMakeFiles/core.dir/src/log.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/core.dir/src/log.cpp.s"
	cd /home/zahorack/projects/c_projects_stm/sprinter/cmake-build-debug/sprinter/core && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zahorack/projects/c_projects_stm/sprinter/sprinter/core/src/log.cpp -o CMakeFiles/core.dir/src/log.cpp.s

sprinter/core/CMakeFiles/core.dir/src/scheduler/add_remove_replace.cpp.obj: sprinter/core/CMakeFiles/core.dir/flags.make
sprinter/core/CMakeFiles/core.dir/src/scheduler/add_remove_replace.cpp.obj: ../sprinter/core/src/scheduler/add_remove_replace.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zahorack/projects/c_projects_stm/sprinter/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object sprinter/core/CMakeFiles/core.dir/src/scheduler/add_remove_replace.cpp.obj"
	cd /home/zahorack/projects/c_projects_stm/sprinter/cmake-build-debug/sprinter/core && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/core.dir/src/scheduler/add_remove_replace.cpp.obj -c /home/zahorack/projects/c_projects_stm/sprinter/sprinter/core/src/scheduler/add_remove_replace.cpp

sprinter/core/CMakeFiles/core.dir/src/scheduler/add_remove_replace.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/core.dir/src/scheduler/add_remove_replace.cpp.i"
	cd /home/zahorack/projects/c_projects_stm/sprinter/cmake-build-debug/sprinter/core && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zahorack/projects/c_projects_stm/sprinter/sprinter/core/src/scheduler/add_remove_replace.cpp > CMakeFiles/core.dir/src/scheduler/add_remove_replace.cpp.i

sprinter/core/CMakeFiles/core.dir/src/scheduler/add_remove_replace.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/core.dir/src/scheduler/add_remove_replace.cpp.s"
	cd /home/zahorack/projects/c_projects_stm/sprinter/cmake-build-debug/sprinter/core && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zahorack/projects/c_projects_stm/sprinter/sprinter/core/src/scheduler/add_remove_replace.cpp -o CMakeFiles/core.dir/src/scheduler/add_remove_replace.cpp.s

sprinter/core/CMakeFiles/core.dir/src/scheduler/schedule.cpp.obj: sprinter/core/CMakeFiles/core.dir/flags.make
sprinter/core/CMakeFiles/core.dir/src/scheduler/schedule.cpp.obj: ../sprinter/core/src/scheduler/schedule.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zahorack/projects/c_projects_stm/sprinter/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object sprinter/core/CMakeFiles/core.dir/src/scheduler/schedule.cpp.obj"
	cd /home/zahorack/projects/c_projects_stm/sprinter/cmake-build-debug/sprinter/core && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/core.dir/src/scheduler/schedule.cpp.obj -c /home/zahorack/projects/c_projects_stm/sprinter/sprinter/core/src/scheduler/schedule.cpp

sprinter/core/CMakeFiles/core.dir/src/scheduler/schedule.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/core.dir/src/scheduler/schedule.cpp.i"
	cd /home/zahorack/projects/c_projects_stm/sprinter/cmake-build-debug/sprinter/core && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zahorack/projects/c_projects_stm/sprinter/sprinter/core/src/scheduler/schedule.cpp > CMakeFiles/core.dir/src/scheduler/schedule.cpp.i

sprinter/core/CMakeFiles/core.dir/src/scheduler/schedule.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/core.dir/src/scheduler/schedule.cpp.s"
	cd /home/zahorack/projects/c_projects_stm/sprinter/cmake-build-debug/sprinter/core && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zahorack/projects/c_projects_stm/sprinter/sprinter/core/src/scheduler/schedule.cpp -o CMakeFiles/core.dir/src/scheduler/schedule.cpp.s

sprinter/core/CMakeFiles/core.dir/src/scheduler/scheduler_state.cpp.obj: sprinter/core/CMakeFiles/core.dir/flags.make
sprinter/core/CMakeFiles/core.dir/src/scheduler/scheduler_state.cpp.obj: ../sprinter/core/src/scheduler/scheduler_state.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zahorack/projects/c_projects_stm/sprinter/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object sprinter/core/CMakeFiles/core.dir/src/scheduler/scheduler_state.cpp.obj"
	cd /home/zahorack/projects/c_projects_stm/sprinter/cmake-build-debug/sprinter/core && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/core.dir/src/scheduler/scheduler_state.cpp.obj -c /home/zahorack/projects/c_projects_stm/sprinter/sprinter/core/src/scheduler/scheduler_state.cpp

sprinter/core/CMakeFiles/core.dir/src/scheduler/scheduler_state.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/core.dir/src/scheduler/scheduler_state.cpp.i"
	cd /home/zahorack/projects/c_projects_stm/sprinter/cmake-build-debug/sprinter/core && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zahorack/projects/c_projects_stm/sprinter/sprinter/core/src/scheduler/scheduler_state.cpp > CMakeFiles/core.dir/src/scheduler/scheduler_state.cpp.i

sprinter/core/CMakeFiles/core.dir/src/scheduler/scheduler_state.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/core.dir/src/scheduler/scheduler_state.cpp.s"
	cd /home/zahorack/projects/c_projects_stm/sprinter/cmake-build-debug/sprinter/core && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zahorack/projects/c_projects_stm/sprinter/sprinter/core/src/scheduler/scheduler_state.cpp -o CMakeFiles/core.dir/src/scheduler/scheduler_state.cpp.s

sprinter/core/CMakeFiles/core.dir/src/scheduler/transfer_continue.cpp.obj: sprinter/core/CMakeFiles/core.dir/flags.make
sprinter/core/CMakeFiles/core.dir/src/scheduler/transfer_continue.cpp.obj: ../sprinter/core/src/scheduler/transfer_continue.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zahorack/projects/c_projects_stm/sprinter/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object sprinter/core/CMakeFiles/core.dir/src/scheduler/transfer_continue.cpp.obj"
	cd /home/zahorack/projects/c_projects_stm/sprinter/cmake-build-debug/sprinter/core && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/core.dir/src/scheduler/transfer_continue.cpp.obj -c /home/zahorack/projects/c_projects_stm/sprinter/sprinter/core/src/scheduler/transfer_continue.cpp

sprinter/core/CMakeFiles/core.dir/src/scheduler/transfer_continue.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/core.dir/src/scheduler/transfer_continue.cpp.i"
	cd /home/zahorack/projects/c_projects_stm/sprinter/cmake-build-debug/sprinter/core && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zahorack/projects/c_projects_stm/sprinter/sprinter/core/src/scheduler/transfer_continue.cpp > CMakeFiles/core.dir/src/scheduler/transfer_continue.cpp.i

sprinter/core/CMakeFiles/core.dir/src/scheduler/transfer_continue.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/core.dir/src/scheduler/transfer_continue.cpp.s"
	cd /home/zahorack/projects/c_projects_stm/sprinter/cmake-build-debug/sprinter/core && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zahorack/projects/c_projects_stm/sprinter/sprinter/core/src/scheduler/transfer_continue.cpp -o CMakeFiles/core.dir/src/scheduler/transfer_continue.cpp.s

sprinter/core/CMakeFiles/core.dir/src/task.cpp.obj: sprinter/core/CMakeFiles/core.dir/flags.make
sprinter/core/CMakeFiles/core.dir/src/task.cpp.obj: ../sprinter/core/src/task.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zahorack/projects/c_projects_stm/sprinter/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object sprinter/core/CMakeFiles/core.dir/src/task.cpp.obj"
	cd /home/zahorack/projects/c_projects_stm/sprinter/cmake-build-debug/sprinter/core && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/core.dir/src/task.cpp.obj -c /home/zahorack/projects/c_projects_stm/sprinter/sprinter/core/src/task.cpp

sprinter/core/CMakeFiles/core.dir/src/task.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/core.dir/src/task.cpp.i"
	cd /home/zahorack/projects/c_projects_stm/sprinter/cmake-build-debug/sprinter/core && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zahorack/projects/c_projects_stm/sprinter/sprinter/core/src/task.cpp > CMakeFiles/core.dir/src/task.cpp.i

sprinter/core/CMakeFiles/core.dir/src/task.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/core.dir/src/task.cpp.s"
	cd /home/zahorack/projects/c_projects_stm/sprinter/cmake-build-debug/sprinter/core && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zahorack/projects/c_projects_stm/sprinter/sprinter/core/src/task.cpp -o CMakeFiles/core.dir/src/task.cpp.s

sprinter/core/CMakeFiles/core.dir/src/time.cpp.obj: sprinter/core/CMakeFiles/core.dir/flags.make
sprinter/core/CMakeFiles/core.dir/src/time.cpp.obj: ../sprinter/core/src/time.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zahorack/projects/c_projects_stm/sprinter/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object sprinter/core/CMakeFiles/core.dir/src/time.cpp.obj"
	cd /home/zahorack/projects/c_projects_stm/sprinter/cmake-build-debug/sprinter/core && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/core.dir/src/time.cpp.obj -c /home/zahorack/projects/c_projects_stm/sprinter/sprinter/core/src/time.cpp

sprinter/core/CMakeFiles/core.dir/src/time.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/core.dir/src/time.cpp.i"
	cd /home/zahorack/projects/c_projects_stm/sprinter/cmake-build-debug/sprinter/core && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zahorack/projects/c_projects_stm/sprinter/sprinter/core/src/time.cpp > CMakeFiles/core.dir/src/time.cpp.i

sprinter/core/CMakeFiles/core.dir/src/time.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/core.dir/src/time.cpp.s"
	cd /home/zahorack/projects/c_projects_stm/sprinter/cmake-build-debug/sprinter/core && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zahorack/projects/c_projects_stm/sprinter/sprinter/core/src/time.cpp -o CMakeFiles/core.dir/src/time.cpp.s

# Object files for target core
core_OBJECTS = \
"CMakeFiles/core.dir/src/log.cpp.obj" \
"CMakeFiles/core.dir/src/scheduler/add_remove_replace.cpp.obj" \
"CMakeFiles/core.dir/src/scheduler/schedule.cpp.obj" \
"CMakeFiles/core.dir/src/scheduler/scheduler_state.cpp.obj" \
"CMakeFiles/core.dir/src/scheduler/transfer_continue.cpp.obj" \
"CMakeFiles/core.dir/src/task.cpp.obj" \
"CMakeFiles/core.dir/src/time.cpp.obj"

# External object files for target core
core_EXTERNAL_OBJECTS =

sprinter/core/libcore.a: sprinter/core/CMakeFiles/core.dir/src/log.cpp.obj
sprinter/core/libcore.a: sprinter/core/CMakeFiles/core.dir/src/scheduler/add_remove_replace.cpp.obj
sprinter/core/libcore.a: sprinter/core/CMakeFiles/core.dir/src/scheduler/schedule.cpp.obj
sprinter/core/libcore.a: sprinter/core/CMakeFiles/core.dir/src/scheduler/scheduler_state.cpp.obj
sprinter/core/libcore.a: sprinter/core/CMakeFiles/core.dir/src/scheduler/transfer_continue.cpp.obj
sprinter/core/libcore.a: sprinter/core/CMakeFiles/core.dir/src/task.cpp.obj
sprinter/core/libcore.a: sprinter/core/CMakeFiles/core.dir/src/time.cpp.obj
sprinter/core/libcore.a: sprinter/core/CMakeFiles/core.dir/build.make
sprinter/core/libcore.a: sprinter/core/CMakeFiles/core.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zahorack/projects/c_projects_stm/sprinter/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Linking CXX static library libcore.a"
	cd /home/zahorack/projects/c_projects_stm/sprinter/cmake-build-debug/sprinter/core && $(CMAKE_COMMAND) -P CMakeFiles/core.dir/cmake_clean_target.cmake
	cd /home/zahorack/projects/c_projects_stm/sprinter/cmake-build-debug/sprinter/core && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/core.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
sprinter/core/CMakeFiles/core.dir/build: sprinter/core/libcore.a

.PHONY : sprinter/core/CMakeFiles/core.dir/build

sprinter/core/CMakeFiles/core.dir/clean:
	cd /home/zahorack/projects/c_projects_stm/sprinter/cmake-build-debug/sprinter/core && $(CMAKE_COMMAND) -P CMakeFiles/core.dir/cmake_clean.cmake
.PHONY : sprinter/core/CMakeFiles/core.dir/clean

sprinter/core/CMakeFiles/core.dir/depend:
	cd /home/zahorack/projects/c_projects_stm/sprinter/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zahorack/projects/c_projects_stm/sprinter /home/zahorack/projects/c_projects_stm/sprinter/sprinter/core /home/zahorack/projects/c_projects_stm/sprinter/cmake-build-debug /home/zahorack/projects/c_projects_stm/sprinter/cmake-build-debug/sprinter/core /home/zahorack/projects/c_projects_stm/sprinter/cmake-build-debug/sprinter/core/CMakeFiles/core.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : sprinter/core/CMakeFiles/core.dir/depend

