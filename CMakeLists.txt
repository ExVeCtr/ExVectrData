cmake_minimum_required(VERSION 3.0.0)
project(ExVectrData VERSION 0.1.0)

file(GLOB_RECURSE SRC_FILES src/*.cpp)
add_library(${PROJECT_NAME} ${SRC_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC include/)

function(addExVectrDependency libName)
    target_include_directories(${PROJECT_NAME} PUBLIC ../${libName}/include/)
endfunction()

addExVectrDependency(ExVectrCore)
addExVectrDependency(ExVectrMath)
#addExVectrDependency(ExVectrHAL)