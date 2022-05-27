message( ${CMAKE_MODULE_PATH} )
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package (GeographicLib REQUIRED)

include_directories(${GeographicLib_INCLUDE_DIRS})
list(APPEND ALL_TARGET_LIBRARIES ${GeographicLib_LIBRARIES})