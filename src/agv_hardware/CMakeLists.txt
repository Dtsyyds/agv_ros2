cmake_minimum_required(VERSION 3.12)
project(agv_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 基础依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(jsoncpp REQUIRED)

# ============ OpenSSL 配置（先注释掉，使用非TLS版本）============
# find_package(OpenSSL REQUIRED)
# message(STATUS "OpenSSL version: ${OPENSSL_VERSION}")

# 查找 Boost（websocketpp 需要）
find_package(Boost REQUIRED COMPONENTS system thread)

# 添加 external 子目录，里面定义了 eigen target 
add_subdirectory(external)

# 添加 lib 子目录，里面定义了 xCoreSDK / xMateModel 等 target
add_subdirectory(lib)

# 添加头文件路径
include_directories(include)
include_directories(/usr/include/jsoncpp)
# include_directories(${OPENSSL_INCLUDE_DIR})  # 暂时注释

# ============ WebSocketPP 配置 ============
# 由于 websocketpp 可能没有 cmake 配置文件，我们手动包含
find_path(WEBSOCKETPP_INCLUDE_DIR
  NAMES websocketpp/client.hpp
  PATHS
    /usr/include
    /usr/local/include
    /opt/local/include
)

if(NOT WEBSOCKETPP_INCLUDE_DIR)
  # 从 GitHub 下载
  message(STATUS "websocketpp not found, downloading from GitHub...")
  include(FetchContent)
  
  FetchContent_Declare(
    websocketpp
    GIT_REPOSITORY https://github.com/zaphoyd/websocketpp.git
    GIT_TAG 0.8.2
  )
  
  FetchContent_GetProperties(websocketpp)
  if(NOT websocketpp_POPULATED)
    FetchContent_Populate(websocketpp)
    set(WEBSOCKETPP_INCLUDE_DIR ${websocketpp_SOURCE_DIR})
  endif()
endif()

message(STATUS "websocketpp include dir: ${WEBSOCKETPP_INCLUDE_DIR}")
include_directories(${WEBSOCKETPP_INCLUDE_DIR})

# ============ ASIO 配置 ============
find_path(ASIO_INCLUDE_DIR
  NAMES asio.hpp
  PATHS
    /usr/include
    /usr/local/include
    /opt/local/include
)

if(NOT ASIO_INCLUDE_DIR)
  # 从 websocketpp 的 vendor 目录中查找
  find_path(ASIO_INCLUDE_DIR
    NAMES asio.hpp
    PATHS ${WEBSOCKETPP_INCLUDE_DIR}/vendor/asio/asio/include
  )
  
  if(NOT ASIO_INCLUDE_DIR)
    # 下载 ASIO
    message(STATUS "ASIO not found, downloading...")
    include(FetchContent)
    
    FetchContent_Declare(
      asio
      GIT_REPOSITORY https://github.com/chriskohlhoff/asio.git
      GIT_TAG asio-1-28-0
    )
    
    FetchContent_GetProperties(asio)
    if(NOT asio_POPULATED)
      FetchContent_Populate(asio)
      set(ASIO_INCLUDE_DIR ${asio_SOURCE_DIR}/asio/include)
    endif()
  endif()
endif()

message(STATUS "asio include dir: ${ASIO_INCLUDE_DIR}")
include_directories(${ASIO_INCLUDE_DIR})

# 定义一个库
add_library(
  ${PROJECT_NAME}
  SHARED
  src/agv_hardware_interface.cpp
)

# 添加编译定义
target_compile_definitions(${PROJECT_NAME} PRIVATE
  ASIO_STANDALONE
  BOOST_ERROR_CODE_HEADER_ONLY
  # _WEBSOCKETPP_CPP11_STL_  # 如果使用C++11特性，可以添加
)

# ROS2依赖
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  jsoncpp
)

# 普通CMake target依赖
target_link_libraries(${PROJECT_NAME}
  Boost::system
  Boost::thread
  eigen
  Rokae::Rokae
  jsoncpp_lib
  # ${OPENSSL_LIBRARIES}  # 暂时注释，使用非TLS版本
)

# 添加必要的库
target_link_libraries(${PROJECT_NAME}
  pthread
)

# 添加包含目录
target_include_directories(${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${WEBSOCKETPP_INCLUDE_DIR}
  ${ASIO_INCLUDE_DIR}
  /usr/include/jsoncpp
  # ${OPENSSL_INCLUDE_DIR}  # 暂时注释
)

pluginlib_export_plugin_description_file(hardware_interface agv_hardware_interface.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include
  DESTINATION include
)

ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  eigen
  Rokae::Rokae
  jsoncpp
)

ament_package()