include("${_build_cmake_local}")

add_library(PiSubmarine.Bq25792 INTERFACE)
target_include_directories(PiSubmarine.Bq25792 INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/public")

PiSubmarineAddDependency("https://github.com/PiSubmarine/RegUtils" "")
target_link_libraries(PiSubmarine.Bq25792 INTERFACE PiSubmarine.RegUtils)