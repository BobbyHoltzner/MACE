use_cxx11()

file(GLOB lib_srcs *.cpp)
file(GLOB lib_hdrs *.h)

add_library(module_vehicle_MAVLINK ${lib_srcs} ${lib_hdrs})
 
target_link_libraries(module_vehicle_MAVLINK mace_core)
 
install(TARGETS module_vehicle_MAVLINK
  # IMPORTANT: Add the libjpeg library to the "export-set"
  EXPORT MACETargets
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib)
    
    
INSTALL(FILES ${lib_hdrs}
    DESTINATION "${INSTALL_INCLUDE_DIR}/module_vehicle_MAVLINK"
    )
    
