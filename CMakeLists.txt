cmake_minimum_required(VERSION 3.25)
project(physics_engine)

set(CMAKE_CXX_STANDARD 23)

option(GLFW_BUILD_DOCS OFF)
option(GLFW_BUILD_EXAMPLES OFF)
option(GLFW_BUILD_TESTS OFF)
add_subdirectory(C:/builds/glfw-3.3.8 ${PROJECT_NAME}/glfw)

option(ASSIMP_BUILD_ASSIMP_TOOLS OFF)
option(ASSIMP_BUILD_SAMPLES OFF)
option(ASSIMP_BUILD_TESTS OFF)
add_subdirectory(C:/builds/assimp-master ${PROJECT_NAME}/assimp)

set(BOOST_ROOT "C:/builds/boost_1_82_0")
find_package(Boost REQUIRED)

add_subdirectory(C:/builds/URDF_Parser-master ${PROJECT_NAME}/urdf-parser)



set(PHSYX_LIBS
        "PhysX_64.lib"
        "PhysXCommon_64.lib"
        "PhysXCooking_64.lib"
        "PhysXFoundation_64.lib"

        "PhysXPvdSDK_static_64.lib"
        "PhysXVehicle_static_64.lib"
        "PhysXExtensions_static_64.lib"
        "PhysXCharacterKinematic_static_64.lib"
        )

set(PHYSX_ROOT_WIN "C:/builds/PhysX-release-104.2/physx")
set(PHSYX_LIBS_ROOT_DIR "C:/builds/PhysX-release-104.2/physx/bin/win.x86_64.vc143.md/debug")

find_package(CUDA REQUIRED)
message(STATUS "Found CUDA ${CUDA_VERSION_STRING} at ${CUDA_TOOLKIT_ROOT_DIR}")
set(CUDA_INCLUDE "${CUDA_TOOLKIT_ROOT_DIR}/include")

find_package(OpenGL REQUIRED)
message(STATUS "Found OpenGL ${OPENGL_VERSION_STRING} at ${OPENGL_INCLUDE_DIR}")


file(GLOB PROJECT_SOURCES src/*.cpp)
#file(GLOB MESH_SOURCES headers/cell-mesh/*.cpp)

set(glad_source src/glad.c)

include_directories("${PHYSX_ROOT_WIN}/include/"
                    "C:/builds/glfw-3.3.8/include"
                    Headers/
                    "${CUDA_INCLUDE}"
                    "C:/builds/assimp-master/include"
                    "C:/builds/URDF_Parser-master/include")

if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W3")
endif()

add_definitions(-DGLFW_INCLUDE_NONE
        -DPROJECT_SOURCE_DIR=\"${PROJECT_SOURCE_DIR}\")

add_executable(${PROJECT_NAME} main.cpp ${PROJECT_SOURCES} ${glad_source})
#add_executable(${PROJECT_NAME} main.cpp ${PROJECT_SOURCES} ${glad_source} ${MESH_SOURCES})

target_link_libraries(${PROJECT_NAME} PUBLIC glfw assimp urdfparser ${OPENGL_LIBRARIES} ${cuda_lib1} ${cuda_lib2})

# Copy cuda dll to build directory
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CUDA_TOOLKIT_ROOT_DIR}/bin/cudart64_12.dll"
        $<TARGET_FILE_DIR:${PROJECT_NAME}>)

foreach (PHYSX_LIB ${PHSYX_LIBS})
    target_link_libraries(${PROJECT_NAME} PUBLIC "${PHSYX_LIBS_ROOT_DIR}/${PHYSX_LIB}")
endforeach ()
