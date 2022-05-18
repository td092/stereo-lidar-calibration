find_package(PCL  REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

FIND_PACKAGE( Boost )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR})

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

FIND_PACKAGE(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIRS})

include_directories(../include)

list(APPEND LINK_LIBS 
    cbdetect
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${OpenCV_LIBS}
    ${YAML_CPP_LIBRARIES}
    ${PCL_LIBRARIES}  
    ${CERES_LIBRARIES} 
    ${Sophus_LIBRARIES} 
    ${GTEST_BOTH_LIBRARIES}
    fmt
    pthread
)

file(GLOB SOURCE_FILES ../src/*.cpp)
file(GLOB HEADER_FILES ../include/*.hpp )

add_executable(test_config test_config.cpp ${SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(test_config ${LINK_LIBS} )

