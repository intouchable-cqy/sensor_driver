cmake_minimum_required(VERSION 3.0.2)
project(gnss_driver)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 启用 RPATH 并指定路径，解决动态库加载的问题
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH $ORIGIN)

# 找到必要的 ROS 包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  message_generation
  tf2
  tf2_geometry_msgs
  tf2_ros
)

# 其他依赖项
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread filesystem date_time coroutine context)
# find_package(Threads REQUIRED)
# find_package(rosconsole_bridge REQUIRED)

# 包含头文件目录
include_directories(
  include 
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# 定义可执行文件
add_executable(${PROJECT_NAME} src/test.cpp src/ProtocolNmea.cpp)

# 链接库
target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES}         # 自动链接所有 ROS 依赖
  ${Boost_LIBRARIES}          # Boost 依赖
  # Threads::Threads            # 多线程库
  pthread                     # POSIX 线程库
  # rosconsole_bridge           # rosconsole_bridge 库
)

# 安装规则
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
