set(project_name "control_optimizer")
project(${project_name})
# 设置要排除的目录
set(EXCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/old*")

# 查找cpp文件，排除old目录
file(GLOB_RECURSE ALL_CPP_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
foreach(FILE ${ALL_CPP_FILES})
    if(NOT ${FILE} MATCHES "^${EXCLUDE_DIR}")
        list(APPEND CPP_FILES ${FILE})
    endif()
endforeach()

# 打印包含cpp文件的目录
foreach(CPP_FILE ${CPP_FILES})
    get_filename_component(DIR ${CPP_FILE} DIRECTORY)
    message(STATUS "Found directory with cpp files: ${DIR}")
endforeach()

# 创建静态库
add_library(${project_name} STATIC ${CPP_FILES})

# 添加头文件目录
target_include_directories(${project_name} PRIVATE ${CMAKE_SOURCE_DIR}/modules/${project_name})
target_include_directories(${project_name} PRIVATE ${CMAKE_SOURCE_DIR}/modules)

# 链接外部库到模块库
target_link_libraries(${project_name} PRIVATE third_party_libs)