cmake_minimum_required(VERSION 3.20)

include(FetchContent)

project(PiSubmarine.Bq25792.Test LANGUAGES CXX)

set(PiSubmarine.Bq25792.Test.Sources 
	"PiSubmarine/Bq25792Test.cpp")

enable_testing()

# spdlog
find_package(spdlog)
find_package(gtest)

add_executable(PiSubmarine.Bq25792.Test ${PiSubmarine.Bq25792.Test.Sources})
target_compile_features(PiSubmarine.Bq25792.Test PRIVATE cxx_std_23)
target_link_libraries(PiSubmarine.Bq25792.Test PRIVATE PiSubmarine.Bq25792)
target_link_libraries(PiSubmarine.Bq25792.Test PRIVATE spdlog::spdlog)
target_link_libraries(PiSubmarine.Bq25792.Test PRIVATE GTest::gtest_main)
include(GoogleTest)
gtest_discover_tests(PiSubmarine.Bq25792.Test)