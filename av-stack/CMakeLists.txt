# av-stack build.
#   Host build (default):  cores + host tests on the in-process ara::com shim (no ara/ROS)
#                          — this is Path A, the verified closed-loop test.
#   AP target build:       -DAP_BACKEND=adaptive-autosar builds the four Adaptive
#                          Applications + safe_stop on real ara::com (Adaptive-AUTOSAR,
#                          SOME/IP) with AP_HAVE_ARA, and the CARLA gateway as the ROS 2
#                          (lwrcl/CycloneDDS) <-> ara::com bridge.
cmake_minimum_required(VERSION 3.16)
project(av_stack_ap CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra)
set(AP_BACKEND "off" CACHE STRING "off | adaptive-autosar")

set(SRC ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(AV_INCLUDES
  ${SRC}/av_common/include ${SRC}/av_ap/include
  ${SRC}/av_perception/include ${SRC}/av_localization/include
  ${SRC}/av_planning/include ${SRC}/av_control/include)

# ROS/middleware-free algorithm cores (always built).
add_library(av_cores STATIC
  ${SRC}/av_perception/src/perception_core.cpp
  ${SRC}/av_localization/src/ekf_localizer.cpp
  ${SRC}/av_planning/src/planner_core.cpp
  ${SRC}/av_control/src/controller_core.cpp)
target_include_directories(av_cores PUBLIC ${AV_INCLUDES})

# Host tests (no lwrcl needed).
enable_testing()
add_executable(test_soa_pipeline ${SRC}/av_integration_tests/test/test_soa_pipeline.cpp)
target_include_directories(test_soa_pipeline PRIVATE ${AV_INCLUDES})
target_link_libraries(test_soa_pipeline av_cores)
add_test(NAME test_soa_pipeline COMMAND test_soa_pipeline)

add_executable(test_ap_integration ${SRC}/av_ap/test/test_ap_integration.cpp)
target_include_directories(test_ap_integration PRIVATE ${AV_INCLUDES})
target_link_libraries(test_ap_integration av_cores)
add_test(NAME test_ap_integration COMMAND test_ap_integration)

# AP target: AAs on ara::com (SOME/IP); gateway = ROS 2 (lwrcl/CycloneDDS) <-> ara::com bridge.
if(AP_BACKEND STREQUAL "adaptive-autosar")
  message(STATUS "AP target: AAs on ara::com/SOME/IP; gateway = ROS2/lwrcl <-> ara::com bridge")

  # --- AP Functional Clusters from Adaptive-AUTOSAR (AdaptiveAutosarAP::ara_*) ---
  # Needs /opt/autosar-ap on CMAKE_PREFIX_PATH (set in build_ap.sh).
  find_package(AdaptiveAutosarAP REQUIRED)
  set(ARA_LIBS
    AdaptiveAutosarAP::ara_core AdaptiveAutosarAP::ara_com
    AdaptiveAutosarAP::ara_log  AdaptiveAutosarAP::ara_exec
    AdaptiveAutosarAP::ara_phm)

  # --- lwrcl (CycloneDDS) + ROS message libs, for the GATEWAY's ROS 2 side only ---
  set(LWRCL_PREFIX      "/opt/cyclonedds-libs" CACHE PATH "lwrcl (cyclonedds) install prefix")
  set(CYCLONEDDS_PREFIX "/opt/cyclonedds"      CACHE PATH "CycloneDDS runtime prefix")
  link_directories(${LWRCL_PREFIX}/lib ${CYCLONEDDS_PREFIX}/lib)
  set(GW_ROS_INCLUDES ${LWRCL_PREFIX}/include ${CYCLONEDDS_PREFIX}/include)
  set(GW_ROS_LIBS lwrcl carla_msgs sensor_msgs nav_msgs geometry_msgs std_msgs
                  builtin_interfaces ddscxx ddsc pthread)

  # Adaptive Applications: real ara::com only (SOME/IP), no ROS.
  foreach(app localization perception planning control)
    add_executable(${app}_app ${SRC}/av_${app}/src/${app}_app_main.cpp)
    target_include_directories(${app}_app PRIVATE ${AV_INCLUDES})
    target_compile_definitions(${app}_app PRIVATE AP_HAVE_ARA)
    target_link_libraries(${app}_app PRIVATE av_cores ${ARA_LIBS})
  endforeach()

  add_executable(safe_stop_app ${SRC}/av_control/src/safe_stop_app_main.cpp)
  target_include_directories(safe_stop_app PRIVATE ${AV_INCLUDES})
  target_compile_definitions(safe_stop_app PRIVATE AP_HAVE_ARA)
  target_link_libraries(safe_stop_app PRIVATE av_cores ${ARA_LIBS})

  # Gateway: ara::com (to the AAs, SOME/IP) + lwrcl/CycloneDDS (to CARLA, ROS 2 wire).
  add_executable(carla_gateway ${SRC}/av_gateway/src/carla_gateway_node.cpp)
  target_include_directories(carla_gateway PRIVATE ${AV_INCLUDES} ${GW_ROS_INCLUDES})
  target_compile_definitions(carla_gateway PRIVATE AP_HAVE_ARA)
  target_link_libraries(carla_gateway PRIVATE av_cores ${ARA_LIBS} ${GW_ROS_LIBS})

  install(TARGETS localization_app perception_app planning_app control_app
                  safe_stop_app carla_gateway RUNTIME DESTINATION bin)
  install(DIRECTORY config/ DESTINATION share/av_stack/config)
endif()
