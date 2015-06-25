# New TI-RPC  Cmake

# Current version as of Fedora 16.  Not tested with earlier.

# Build configure options
option (USE_GSS "enable RPCSEC_GSS support" ON)
option(TIRPC_EPOLL "platform supports EPOLL or emulation" ON)

# MSPAC support -lwbclient link flag
option(_MSPAC_SUPPORT "enable mspac Winbind support" OFF)

if(TIRPC_EPOLL)
  add_definitions(-DTIRPC_EPOLL)
endif(TIRPC_EPOLL)
cmake_minimum_required(VERSION 3.0)

if(USE_NFS_MSK)
  add_definitions(-DUSE_RPC_RDMA)
endif(USE_NFS_MSK)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")

set(PRNX_ROOT "/opt/pernixdata")
set(PRNX_INCLUDE "${PRNX_ROOT}/include")
set(PRNX_LIB "${PRNX_ROOT}/lib")
set (CMAKE_PREFIX_PATH ${CMAKE_INSTALL_PREFIX}:${PRNX_ROOT})

string(CONCAT CMAKE_INSTALL_RPATH
  "${CMAKE_INSTALL_PREFIX}/lib:"
  "${PRNX_ROOT}/lib")

project(NTIRPC C)

set(NTIRPC_MAJOR_VERSION 4)
set(NTIRPC_MINOR_VERSION 2)
set(NTIRPC_PATCH_LEVEL 4)
set(NTIRPC_VERSION ${NTIRPC_MAJOR_VERSION}.${NTIRPC_MINOR_VERSION}.${NTIRPC_PATCH_LEVEL})
set(VERSION_COMMENT
  "Full-duplex and bi-directional ONC RPC on TCP."
)


# Choose a shortcut build config

IF(BUILD_CONFIG)
  INCLUDE(
  ${CMAKE_SOURCE_DIR}/cmake/build_configurations/${BUILD_CONFIG}.cmake)
ENDIF()

# Build source locations and parameters

set(ALLOCATOR "jemalloc" CACHE STRING
  "specify the memory allocator to use: jemalloc|tcmalloc|libc")

# Find packages and libs we need for building
include(CheckIncludeFiles)
include(TestBigEndian)

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  set(LINUX ON)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

if(${CMAKE_SYSTEM_NAME} MATCHES "FreeBSD")
  set(FREEBSD ON)
endif(${CMAKE_SYSTEM_NAME} MATCHES "FreeBSD")

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  set(WINDOWS ON)
  if(${CMAKE_CXX_COMPILER_ID} MATCHES "MSVC")
    set(MSVC ON)
  endif(${CMAKE_CXX_COMPILER_ID} MATCHES "MSVC")
endif(${CMAKE_SYSTEM_NAME} MATCHES "Windows")

check_include_files(stdbool.h HAVE_STDBOOL_H)
check_include_files(strings.h HAVE_STRINGS_H)
check_include_files(string.h HAVE_STRING_H)

TEST_BIG_ENDIAN(BIGENDIAN)
if(${BIGENDIAN})
  set(WORDS_BIGENDIAN ON)
else()
  set(WORDS_BIGENDIAN OFF)
endif(${BIGENDIAN})

find_package(Threads REQUIRED)
find_package(Krb5 REQUIRED gssapi)

if(KRB5_FOUND)
  set(HAVE_KRB5 ON)
  set(KRB5_VERSION 194)  # hand code until we do krb5-config --version magic
  set(_HAVE_GSSAPI ON)
endif(KRB5_FOUND)

set(SYSTEM_LIBRARIES ${SYSTEM_LIBRARIES})

if(_MSPAC_SUPPORT)
  find_package(WBclient REQUIRED)
  set(SYSTEM_LIBRARIES ${WBclient_LIBRARIES} ${SYSTEM_LIBRARIES})
endif(_MSPAC_SUPPORT)

if (FREEBSD)
   set(EXTRA_INCLUDE_DIR "/opt/ganesha/include")
else()
  # workaround bug in some include_directories when no extra includes
  set(EXTRA_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/ntirpc/")
endif(FREEBSD)

add_definitions(-DHAVE_CONFIG_H)

if (MSVC)
   add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif(MSVC)

include_directories(
  "${PROJECT_BINARY_DIR}"
  "${PROJECT_SOURCE_DIR}/ntirpc/"
  "${EXTRA_INCLUDE_DIR}"
)

# Find misc system libs
find_library(LIBRT rt)   # extended Pthreads functions
find_library(LIBNSL nsl) # sockets

set(SYSTEM_LIBRARIES
  ${LIBTIRPC_LIBRARIES}
  ${KRB5_LIBRARIES}
  gssapi_krb5
  ${SYSTEM_LIBRARIES}
  ${LIBDL}
  ${CMAKE_THREAD_LIBS_INIT}
  ${LIBRT}
)

add_subdirectory(src)

# display configuration vars

message(STATUS)
message(STATUS "-------------------------------------------------------")
message(STATUS "TIRPC_EPOLL = ${TIRPC_EPOLL}")

#force command line options to be stored in cache
set(_MSPAC_SUPPORT ${_MSPAC_SUPPORT}
  CACHE BOOL
   "compile with MSPAC extensions"
   FORCE)

set(TIRPC_EPOLL ${TIRPC_EPOLL}
  CACHE BOOL
   "platform has EPOLL or emulation"
   FORCE)

# grist files
configure_file(
  "${PROJECT_SOURCE_DIR}/config-h.in.cmake"
  "${PROJECT_BINARY_DIR}/config.h"
)

configure_file(
  "${PROJECT_SOURCE_DIR}/libntirpc.pc.in.cmake"
  "${PROJECT_BINARY_DIR}/libntirpc.pc"
)


########### install files ###############

#install(FILES  libntirpc.pc DESTINATION bin)
install(DIRECTORY "${PROJECT_SOURCE_DIR}/ntirpc" DESTINATION include COMPONENT dev)

#
# CMake module packaging stuff below.
#
include (CMakePackageConfigHelpers)
write_basic_package_version_file(
	"${CMAKE_CURRENT_BINARY_DIR}/NTIRPCConfigVersion.cmake"
	VERSION ${NTIRPC_VERSION}
	COMPATIBILITY AnyNewerVersion)
set(ConfigPackageLocation lib/cmake/NTIRPC)

export(EXPORT NTIRPC
       FILE "${CMAKE_CURRENT_BINARY_DIR}/NTIRPC.cmake")

CONFIGURE_PACKAGE_CONFIG_FILE(cmake/NTIRPCConfig.cmake
                "${CMAKE_CURRENT_BINARY_DIR}/NTIRPCConfig.cmake"
                INSTALL_DESTINATION ${ConfigPackageLocation}
                PATH_VARS CMAKE_INSTALL_PREFIX)

install(EXPORT NTIRPC 
        FILE NTIRPC.cmake
        DESTINATION ${ConfigPackageLocation}
        COMPONENT dev)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/NTIRPCConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/NTIRPCConfigVersion.cmake"
        DESTINATION ${ConfigPackageLocation}
        COMPONENT dev)

set(CPACK_DEB_COMPONENT_INSTALL 1)

set(CPACK_PACKAGE_VERSION ${NTIRPC_VERSION})
set(CPACK_PACKAGING_INSTALL_PREFIX "${PRNX_ROOT}")
set(CPACK_OUTPUT_FILE_PREFIX "${WS_PACKAGES}")
set(CPACK_DEBIAN_PACKAGE_NAME libntirpc)
set(CPACK_PACKAGE_NAME libntirpc)
set(CPACK_DEBIAN_SECTION_NAME Networking)

set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE amd64)
set(CPACK_PACKAGE_ARCHITECTURE amd64)
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libc6 (>= 2.14), libgcc1 (>= 1:4.1.1)")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "support@pernixdata.com")
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION "New transport-independent RPC (TI-RPC)")

set(CPACK_PACKAGE_FILE_NAME ${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${CPACK_PACKAGE_ARCHITECTURE})
set(CPACK_GENERATOR "DEB")
include(CPack)
