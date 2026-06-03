cmake_minimum_required(VERSION 3.14)
project(multimodal_meeting_predictor LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(mmlp
  src/arrival.cpp
  src/geo.cpp
  src/fleet_service.cpp
  src/fleet_index.cpp
  src/graph.cpp
  src/graph_io.cpp
  src/graph_store.cpp
  src/graph_load.cpp
  src/json_util.cpp
  src/matching.cpp
  src/meeting.cpp
  src/motion.cpp
  src/predict.cpp
  src/region_loader.cpp
  src/routing.cpp
  src/spatial_index.cpp
)
target_include_directories(mmlp PUBLIC include)

add_executable(test_fixture tests/test_fixture.cpp)
target_link_libraries(test_fixture PRIVATE mmlp)
target_include_directories(test_fixture PRIVATE tests)

add_executable(test_meeting tests/test_meeting.cpp)
target_link_libraries(test_meeting PRIVATE mmlp)
target_include_directories(test_meeting PRIVATE tests)

enable_testing()
add_test(NAME fixture COMMAND test_fixture)
add_test(NAME meeting COMMAND test_meeting)

add_executable(test_fleet_service tests/test_fleet_service.cpp)
target_link_libraries(test_fleet_service PRIVATE mmlp)
target_include_directories(test_fleet_service PRIVATE tests)
add_test(NAME fleet_service COMMAND test_fleet_service)

add_executable(mmlp_predict apps/mmlp_predict.cpp)
target_link_libraries(mmlp_predict PRIVATE mmlp)

add_executable(mmlp_service apps/mmlp_service.cpp)
target_link_libraries(mmlp_service PRIVATE mmlp)

add_executable(mmlp_build_aux apps/mmlp_build_aux.cpp)
target_link_libraries(mmlp_build_aux PRIVATE mmlp)

add_executable(test_graph_io tests/test_graph_io.cpp)
target_link_libraries(test_graph_io PRIVATE mmlp)
add_test(NAME graph_io COMMAND test_graph_io)
set_tests_properties(graph_io PROPERTIES ENVIRONMENT "MMLP_ROOT=${CMAKE_SOURCE_DIR}")

add_executable(test_region_predict tests/test_region_predict.cpp)
target_link_libraries(test_region_predict PRIVATE mmlp)
add_test(NAME region_predict COMMAND test_region_predict)
set_tests_properties(region_predict PROPERTIES
  ENVIRONMENT "MMLP_GRAPH_PATH=${CMAKE_SOURCE_DIR}/data/graph/china.mmlp.bin"
  LABELS "integration"
)
