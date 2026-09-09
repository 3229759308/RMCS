cmake_minimum_required(VERSION 3.8)
project(week3_astar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 将源文件编译为名叫 astar_test 的可执行程序。
add_executable(astar_test src/astar_test.cpp)

# 配置 OpenCV 头文件目录和链接库。
target_include_directories(astar_test PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(astar_test PRIVATE ${OpenCV_LIBS})

# 安装到 ROS2 能找到的位置。
install(TARGETS astar_test
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
