get_filename_component(_onnxruntime_prefix "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)

set(onnxruntime_FOUND TRUE)
set(onnxruntime_INCLUDE_DIRS "${_onnxruntime_prefix}/include")
set(onnxruntime_LIBRARIES onnxruntime)

if(NOT TARGET onnxruntime)
  add_library(onnxruntime SHARED IMPORTED)
  set_target_properties(onnxruntime PROPERTIES
    IMPORTED_LOCATION "${_onnxruntime_prefix}/lib/libonnxruntime.so.1.19.0"
    INTERFACE_INCLUDE_DIRECTORIES "${onnxruntime_INCLUDE_DIRS}"
  )
endif()

unset(_onnxruntime_prefix)
