include(FetchContent)

FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.8.1
)

FetchContent_MakeAvailable(Catch2)

add_executable(traffic_tests traffic_sim_test.cpp)
target_link_libraries(traffic_tests PRIVATE traffic_core Catch2::Catch2WithMain)
traffic_warnings(traffic_tests)


include(CTest)
include(Catch)
catch_discover_tests(traffic_tests ADD_TAGS_AS_LABELS)
