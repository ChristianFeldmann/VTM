cmake_minimum_required(VERSION 3.11)

project(vtmdecoder)

include(FetchContent)
set(FETCHCONTENT_QUIET FALSE)

# Latest VTM
# UPDATE_DISCONNECTED disables updating, i.e. to get a new version
# of VTM you will have to delete your build/ folder.
# This is necessary because, when a git tag is not provided, the
# git update mechanism always triggers an `update` even if the files
# have not changed. This in turn causes the patch command to be
# executed, which causes an error as the file is no longer as
# expected.
FetchContent_Declare(
  VTM
  GIT_REPOSITORY https://vcgit.hhi.fraunhofer.de/jvet/VVCSoftware_VTM
  GIT_PROGRESS TRUE
  UPDATE_DISCONNECTED TRUE
  PATCH_COMMAND git apply ${CMAKE_CURRENT_SOURCE_DIR}/vtm-cmake.patch
)
FetchContent_MakeAvailable(VTM)

# executable
set( LIB_NAME vtmdecoder )

# get source files
file( GLOB SRC_FILES "src/*.cpp" )

# get include files
file( GLOB INC_FILES "src/*.h" )

# get additional libs for gcc on Ubuntu systems
if( CMAKE_SYSTEM_NAME STREQUAL "Linux" )
  if( CMAKE_CXX_COMPILER_ID STREQUAL "GNU" )
    if( USE_ADDRESS_SANITIZER )
      set( ADDITIONAL_LIBS asan )
    endif()
  endif()
endif()

# NATVIS files for Visual Studio
if( MSVC )
  file( GLOB NATVIS_FILES "../../VisualStudio/*.natvis" )
  # extend the stack size on windows to 2MB
  set( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} /STACK:0x200000" )
endif()

# add executable
add_library( ${LIB_NAME} SHARED  ${SRC_FILES} ${INC_FILES} ${NATVIS_FILES} )
include_directories(${CMAKE_CURRENT_BINARY_DIR})

target_compile_features( ${LIB_NAME} PUBLIC cxx_std_14 )

if( SET_ENABLE_TRACING )
  if( ENABLE_TRACING )
    target_compile_definitions( ${LIB_NAME} PUBLIC ENABLE_TRACING=1 )
  else()
    target_compile_definitions( ${LIB_NAME} PUBLIC ENABLE_TRACING=0 )
  endif()
endif()

if( OpenMP_FOUND )
  if( SET_ENABLE_SPLIT_PARALLELISM )
    if( ENABLE_SPLIT_PARALLELISM )
      target_compile_definitions( ${LIB_NAME} PUBLIC ENABLE_SPLIT_PARALLELISM=1 )
    else()
      target_compile_definitions( ${LIB_NAME} PUBLIC ENABLE_SPLIT_PARALLELISM=0 )
    endif()
  endif()
  if( SET_ENABLE_WPP_PARALLELISM )
    if( ENABLE_WPP_PARALLELISM )
      target_compile_definitions( ${LIB_NAME} PUBLIC ENABLE_WPP_PARALLELISM=1 )
    else()
      target_compile_definitions( ${LIB_NAME} PUBLIC ENABLE_WPP_PARALLELISM=0 )
    endif()
  endif()
else()
  target_compile_definitions( ${LIB_NAME} PUBLIC ENABLE_SPLIT_PARALLELISM=0 )
  target_compile_definitions( ${LIB_NAME} PUBLIC ENABLE_WPP_PARALLELISM=0 )
endif()

if( CMAKE_COMPILER_IS_GNUCC AND BUILD_STATIC )
  set( ADDITIONAL_LIBS ${ADDITIONAL_LIBS} -static -static-libgcc -static-libstdc++ )
  target_compile_definitions( ${LIB_NAME} PUBLIC ENABLE_WPP_STATIC_LINK=1 )
endif()

target_link_libraries( ${LIB_NAME} CommonLib DecoderLib Utilities ${ADDITIONAL_LIBS} )

# lldb custom data formatters
if( XCODE )
  add_dependencies( ${LIB_NAME} Install${PROJECT_NAME}LldbFiles )
endif()

if( CMAKE_SYSTEM_NAME STREQUAL "Linux" )
  add_custom_command( TARGET ${LIB_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy
                                                          $<$<CONFIG:Debug>:${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG}/libvtmdecoder.so>
                                                          $<$<CONFIG:Release>:${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE}/libvtmdecoder.so>
                                                          $<$<CONFIG:RelWithDebInfo>:${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELWITHDEBINFO}/libvtmdecoder.so>
                                                          $<$<CONFIG:MinSizeRel>:${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_MINSIZEREL}/libvtmdecoder.so>
                                                          $<$<CONFIG:Debug>:${CMAKE_SOURCE_DIR}/bin/libvtmdecoder.so>
                                                          $<$<CONFIG:Release>:${CMAKE_SOURCE_DIR}/bin/libvtmdecoder.so>
                                                          $<$<CONFIG:RelWithDebInfo>:${CMAKE_SOURCE_DIR}/bin/libvtmdecoder.so>
                                                          $<$<CONFIG:MinSizeRel>:${CMAKE_SOURCE_DIR}/bin/libvtmdecoder.so> )
endif()

# example: place header files in different folders
source_group( "Natvis Files" FILES ${NATVIS_FILES} )

# set the folder where to place the projects
set_target_properties( ${LIB_NAME}  PROPERTIES FOLDER app LINKER_LANGUAGE CXX )
