# conditional defines for backward-cpp
# backward-cpp adds automatic stack traces and line info when crashing or throwing exceptions

find_package(Libunwind QUIET)
if(LIBUNWIND_FOUND)
    target_compile_definitions(voxeland_map PUBLIC "BACKWARD_HAS_LIBUNWIND=1")
    target_link_libraries(voxeland_server unwind)
endif()

find_package(LibBfd QUIET)
if(LIBBFD_FOUND)
    target_compile_definitions(voxeland_map PUBLIC "BACKWARD_HAS_BFD=1")
    target_link_libraries(voxeland_server bfd)
endif()
