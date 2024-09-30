find_package(kaylordut REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
set(link_libs ${kaylordut_LIBS} ${PCL_LIBRARIES} ${OpenCV_LIBS})


file(GLOB_RECURSE SRC "./*.cpp")

foreach(source_file ${SRC})
	message(STATUS "Found source file: ${source_file}")
endforeach()

add_executable(${PROJECT_NAME} ${SRC})
ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs cv_bridge pcl_ros pcl_conversions)
target_link_libraries(${PROJECT_NAME} ${link_libs})

install(TARGETS
		${PROJECT_NAME}
		EXPORT ${PROJECT_NAME}_export
		ARCHIVE DESTINATION lib/${PROJECT_NAME}
		LIBRARY DESTINATION lib/${PROJECT_NAME}
		RUNTIME DESTINATION lib/${PROJECT_NAME}
)

