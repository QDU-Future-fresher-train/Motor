# CMakeLists.txt for Motor

target_include_directories(xr PUBLIC ${CMAKE_CURRENT_LIST_DIR})

file(GLOB MODULE_MOTOR_SRC CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_LIST_DIR}/*.cpp"
)

target_sources(xr PRIVATE ${MODULE_MOTOR_SRC})