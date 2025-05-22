cmake_minimum_required(VERSION 3.5)
project(scout_description)

# 使用 ament_cmake
find_package(ament_cmake REQUIRED)

# 安裝 launch、meshes、urdf 目錄
install(
  DIRECTORY launch meshes urdf urdf_zyx meshes_zyx me30_STL rviz
  DESTINATION share/${PROJECT_NAME}
)

# 設定 ament_package
ament_package()

