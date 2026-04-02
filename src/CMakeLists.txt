include("${_build_cmake_local}")

add_library(PiSubmarine.Bq25792
        PiSubmarine/Bq25792/Device.cpp
)
target_include_directories(PiSubmarine.Bq25792 PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../public")
PiSubmarineInitTarget(PiSubmarine.Bq25792)

PiSubmarineAddDependency("https://github.com/PiSubmarine/RegUtils" "")
target_link_libraries(PiSubmarine.Bq25792 PUBLIC PiSubmarine.RegUtils)

PiSubmarineAddDependency("https://github.com/PiSubmarine/I2C.Api" "")
target_link_libraries(PiSubmarine.Bq25792 PUBLIC PiSubmarine.I2C.Api)

PiSubmarineAddDependency("https://github.com/PiSubmarine/Units" "")
target_link_libraries(PiSubmarine.Bq25792 PUBLIC PiSubmarine.Units)