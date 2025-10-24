###########################################################################
# CMakeLists.txt generated for component flyball
# Product type: executable
###########################################################################
cmake_minimum_required(VERSION 3.12)
project(flyball)

# Propagate the CMAKE_EXPORT_COMPILE_COMMANDS variable from the 
# environment if it is defined as an environment variable, but not as a 
# CMake variable. This is to work around a bug in CMake 3.19 when the 
# "NMake Makefiles" generator is selected.
if(DEFINED ENV{CMAKE_EXPORT_COMPILE_COMMANDS} AND NOT DEFINED CMAKE_EXPORT_COMPILE_COMMANDS)
    set(CMAKE_EXPORT_COMPILE_COMMANDS $ENV{CMAKE_EXPORT_COMPILE_COMMANDS})
endif()


# Set directory properties.
set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT flyball)

###########################################################################
## Path variables
###########################################################################
# Derive an absolute path to the code generation anchor folder.
get_filename_component(START_DIR .. ABSOLUTE)

# Special directories defined by using CACHE variables can be overridden 
# by setting the variable from the command line, e.g.,
# 
# 	cmake . -DMATLAB_ROOT=/path/to/another/matlab/root
set(MATLAB_ROOT E:/BaiduNetdiskDownload/matlab24 CACHE PATH "")

###########################################################################
## System Libraries
###########################################################################
find_library(FOUND_LIBM m NO_SYSTEM_ENVIRONMENT_PATH PATHS ${CMAKE_C_IMPLICIT_LINK_DIRECTORIES} ${CMAKE_CXX_IMPLICIT_LINK_DIRECTORIES})

###########################################################################
## Target definition and commands
###########################################################################

# When a CMakeLists.txt defines multiple targets that have a common set of 
# tool flags and preprocessor definitions, define variables containing 
# data that is common to all targets defined in the file.
set(FLYBALL_COMMON_DEFINES -DMODEL=flyball
    -DNUMST=2
    -DNCSTATES=2
    -DHAVESTDIO
    -DMODEL_HAS_DYNAMICALLY_LOADED_SFCNS=0
    -DCLASSIC_INTERFACE=0
    -DALLOCATIONFCN=0
    -DTID01EQ=1
    -DTERMFCN=1
    -DONESTEPFCN=1
    -DMAT_FILE=0
    -DMULTI_INSTANCE_CODE=1
    -DINTEGER_CODE=0
    -DMT=0
    -D_CRT_SECURE_NO_WARNINGS)

# Definition of target "flyball_objects".
add_library(flyball_objects OBJECT ${START_DIR}/flyball_ert_rtw/flyball.cpp
    ${START_DIR}/flyball_ert_rtw/flyball_data.cpp)

# Set properties for target "flyball_objects".
set_target_properties(flyball_objects PROPERTIES PREFIX ""
    POSITION_INDEPENDENT_CODE ON)

# Specify language features required for target "flyball_objects".
target_compile_features(flyball_objects PUBLIC cxx_std_11)

# Specify compiler preprocessor definitions for target "flyball_objects".
target_compile_definitions(flyball_objects PRIVATE ${FLYBALL_COMMON_DEFINES})

# Specify include directories for target "flyball_objects".
target_include_directories(flyball_objects PUBLIC $<BUILD_INTERFACE:${START_DIR}>
    $<INSTALL_INTERFACE:$<INSTALL_PREFIX>>
    $<BUILD_INTERFACE:${START_DIR}/flyball_ert_rtw>
    $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/flyball_ert_rtw>
    $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/include>
    $<BUILD_INTERFACE:${MATLAB_ROOT}/simulink/include>
    $<BUILD_INTERFACE:${MATLAB_ROOT}/rtw/c/src>
    $<BUILD_INTERFACE:${MATLAB_ROOT}/rtw/c/src/ext_mode/common>
    $<BUILD_INTERFACE:${MATLAB_ROOT}/rtw/c/ert>)

# Specify library link dependencies for target "flyball_objects". CMake 
# generator expressions are used to create a CMakeLists.txt file that 
# supports multiple platforms with differently named system library 
# dependencies.
target_link_libraries(flyball_objects PUBLIC $<$<BOOL:${FOUND_LIBM}>:m>)


# Definition of target "flyball".
add_executable(flyball  ${START_DIR}/flyball_ert_rtw/ert_main.cpp)

# Set properties for target "flyball".
set_target_properties(flyball PROPERTIES PREFIX ""
    POSITION_INDEPENDENT_CODE ON
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH_USE_LINK_PATH TRUE)

# Specify language features required for target "flyball".
target_compile_features(flyball PUBLIC cxx_std_11)

# Specify compiler preprocessor definitions for target "flyball".
target_compile_definitions(flyball PRIVATE ${FLYBALL_COMMON_DEFINES})

# Specify library link dependencies for target "flyball". CMake generator 
# expressions are used to create a CMakeLists.txt file that supports 
# multiple platforms with differently named system library dependencies.
target_link_libraries(flyball PRIVATE flyball_objects
    $<$<BOOL:${WIN32}>:kernel32>
    $<$<BOOL:${WIN32}>:ws2_32>
    $<$<BOOL:${WIN32}>:mswsock>
    $<$<BOOL:${WIN32}>:advapi32>)

# Extract DWARF debug symbols into a separate file when building 
# executable programs for Apple platforms.
if(APPLE AND (CMAKE_C_COMPILER_ID STREQUAL "AppleClang"))
    string(APPEND _dsymutilcmd "$<IF:$<CONFIG:DEBUG,RELWITHDEBINFO>,"
                "xcrun;dsymutil;$<TARGET_FILE:flyball>;--flat,"
                "true"
        >)
    add_custom_command(TARGET flyball
            POST_BUILD
            COMMAND "${_dsymutilcmd}"
            COMMAND_EXPAND_LISTS
            BYPRODUCTS flyball.dwarf)
endif()


###########################################################################
## Target install rules
###########################################################################

# Install rule for objects of target "flyball_objects" to allow support 
# for SIL/PIL.
install(TARGETS flyball_objects
    EXPORT flyballTargets
    OBJECTS DESTINATION "flyball_ert_rtw")

# Install rule for executable target "flyball". The "OPTIONAL" keyword is 
# required to support SIL/PIL when build of the final executable is 
# suppressed.
install(TARGETS flyball
    RUNTIME DESTINATION "."
    OPTIONAL)

# Install rule for Microsoft PDB debug symbols, if required.
if(CMAKE_C_LINKER_SUPPORTS_PDB)
    install(FILES $<TARGET_PDB_FILE:flyball>
        DESTINATION .
        OPTIONAL
        CONFIGURATIONS Debug RelWithDebInfo)
endif()

# Install rule for Apple DWARF debug symbols, if required.
if(APPLE AND (CMAKE_C_COMPILER_ID STREQUAL "AppleClang"))
    install(FILES $<TARGET_FILE_DIR:flyball>/flyball.dwarf
        DESTINATION .
        OPTIONAL
        CONFIGURATIONS Debug RelWithDebInfo)
endif()

# Write a rule that generates a wrapper around exported targets to 
# preserve tokenization of "special" directories (e.g., MATLAB_ROOT). This 
# avoids hard-coding absolute paths in the CMake file with the code used 
# to import the targets, and avoids errors when include paths that do not 
# exist in the current filesystem are defined, for example, after 
# relocating code using PackNGo.
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/flyball.cmake" [=[include("${CMAKE_CURRENT_LIST_DIR}/flyballTargets.cmake")]=] \n)
file(APPEND "${CMAKE_CURRENT_BINARY_DIR}/flyball.cmake" [=[set(FLYBALL_OBJECTS_TOK_INC_DIRS_ALL]=] \n
    [=[    ${MATLAB_ROOT}/extern/include]=] \n
    [=[    ${MATLAB_ROOT}/simulink/include]=] \n
    [=[    ${MATLAB_ROOT}/rtw/c/src]=] \n
    [=[    ${MATLAB_ROOT}/rtw/c/src/ext_mode/common]=] \n
    [=[    ${MATLAB_ROOT}/rtw/c/ert)]=] \n
    [=[foreach(TOKDIR_LOOP IN LISTS FLYBALL_OBJECTS_TOK_INC_DIRS_ALL)]=] \n
    [=[    if(IS_DIRECTORY ${TOKDIR_LOOP})]=] \n
    [=[        list(APPEND FLYBALL_OBJECTS_TOK_INC_DIRS ${TOKDIR_LOOP})]=] \n
    [=[    endif()]=] \n
    [=[endforeach()]=] \n
    [=[target_include_directories(flyball::flyball_objects INTERFACE ${FLYBALL_OBJECTS_TOK_INC_DIRS})]=] \n)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/flyball.cmake"
    DESTINATION "flyball_ert_rtw/export")

# Generate and install a file that allows the targets generated from this 
# CMakeLists.txt file to be imported into another project.
install(EXPORT flyballTargets NAMESPACE flyball:: DESTINATION flyball_ert_rtw/export)

###########################################################################
## Build success message
###########################################################################
add_custom_command(TARGET flyball POST_BUILD
     COMMAND ${CMAKE_COMMAND} -E cmake_echo_color --cyan "\\#\\#\\# Created executable: $<TARGET_FILE:flyball>")

###########################################################################
## Call toolchain hook function if defined
###########################################################################
if(COMMAND toolchain_target_hook)
    toolchain_target_hook(flyball)
endif()
