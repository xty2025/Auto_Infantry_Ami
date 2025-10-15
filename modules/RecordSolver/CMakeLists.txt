# 设置模块名称
project(recordsolver)

# 查找所有包含cpp文件的子目录
file(GLOB_RECURSE CPP_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

# 打印包含cpp文件的目录
foreach(CPP_FILE ${CPP_FILES})
    get_filename_component(DIR ${CPP_FILE} DIRECTORY)
    message(STATUS "Found directory with cpp files: ${DIR}")
endforeach()

# 添加源文件
file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

# 创建静态库
add_library(recordsolver STATIC ${SOURCES})

# 添加头文件目录
target_include_directories(recordsolver PRIVATE ${CMAKE_SOURCE_DIR}/modules/RecordSolver)
target_include_directories(recordsolver PRIVATE ${CMAKE_SOURCE_DIR}/modules)

# 链接外部库到模块库
target_link_libraries(recordsolver PRIVATE third_party_libs)
