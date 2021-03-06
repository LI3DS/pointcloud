
set ( PC_SOURCES
        hashtable.c 
        stringbuffer.c      
        pc_bytes.c       
        pc_dimstats.c      
        pc_filter.c    
        pc_interp.c
        pc_mem.c 
        pc_patch.c
        pc_patch_dimensional.c
        pc_patch_ght.c
        pc_patch_lazperf.c
        pc_patch_uncompressed.c
        pc_point.c
        pc_pointlist.c
        pc_schema.c
        pc_sort.c
        pc_stats.c
        pc_util.c
        pc_val.c
        )

set ( LI_SOURCES
        li_matrix.c
        li_patch.c
        li_point.c
        li_box4d.c
        )

set ( LAZPERF_SOURCES
        lazperf_adapter.cpp
        )

set ( PC_HEADERS
        hashtable.h           
        stringbuffer.h    
        pc_api.h  
        pc_api_internal.h
        pc_config.h
        )

set ( LI_HEADERS
        li_api.h
        li_api_internal.h
        )

include_directories(${LIBXML2_INCLUDE_DIRS})
include_directories(${ZLIB_INCLUDE_DIRS})

add_library (libpc-static STATIC ${PC_SOURCES} ${PC_HEADERS} ${LI_SOURCES} ${LI_HEADERS})
add_library (liblazperf-static STATIC ${LAZPERF_SOURCES})

set_target_properties (libpc-static 
  PROPERTIES
    OUTPUT_NAME "pc"
    PREFIX "lib"
    CLEAN_DIRECT_OUTPUT 1
    COMPILE_FLAGS "-fPIC"
  )

set_target_properties (liblazperf-static
  PROPERTIES
    OUTPUT_NAME "lazperf"
    PREFIX "lib"
    CLEAN_DIRECT_OUTPUT 1
    COMPILE_FLAGS "-fPIC -std=c++0x"
  )

# TODO: remove ${LIBXML2_INCLUDE_DIR} eventually,
# it's deprecated since CMake 3.7.2
target_include_directories(libpc-static PUBLIC ${LIBXML2_INCLUDE_DIR} ${LIBXML2_INCLUDE_DIRS})
target_include_directories(libpc-static PUBLIC ${ZLIB_INCLUDE_DIRS})

target_link_libraries (libpc-static ${LIBXML2_LIBRARIES})
target_link_libraries (libpc-static ${ZLIB_LIBRARIES})
target_link_libraries (libpc-static m)
if (LIBGHT_FOUND)
  target_link_libraries (libpc-static ${LIBGHT_LIBRARY})
endif (LIBGHT_FOUND)
if (LAZPERF_FOUND)
  target_link_libraries (libpc-static liblazperf-static)
endif (LAZPERF_FOUND)

if (WITH_TESTS)
    add_subdirectory (cunit)
endif (WITH_TESTS)
