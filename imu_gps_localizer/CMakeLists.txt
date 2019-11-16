
find_package(Eigen3  REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}_lib
	src/imu_gps_localizer.cpp
)

target_link_libraries(${PROJECT_NAME}_lib
	${EIGEN3_LIBS}
)