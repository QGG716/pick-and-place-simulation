cmake_minimum_required(VERSION 3.18)
project(unloading_tesseract_ompl LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(tesseract 0.35.0 EXACT REQUIRED COMPONENTS environment collision)
find_package(ompl REQUIRED)
find_package(nlohmann_json REQUIRED)
add_executable(unloading_tesseract_ompl worker.cpp)
get_filename_component(TESSERACT_NATIVE_PREFIX "${tesseract_DIR}/../../.." ABSOLUTE)
target_compile_definitions(unloading_tesseract_ompl PRIVATE
  TESSERACT_PLUGIN_LIBDIR="${TESSERACT_NATIVE_PREFIX}/lib")
target_link_libraries(unloading_tesseract_ompl PRIVATE tesseract::environment tesseract::collision
  nlohmann_json::nlohmann_json ${OMPL_LIBRARIES})
target_include_directories(unloading_tesseract_ompl PRIVATE ${OMPL_INCLUDE_DIRS})
install(TARGETS unloading_tesseract_ompl RUNTIME DESTINATION bin)
