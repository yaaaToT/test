cmake_minimum_required(VERSION 3.28)
project(rune)
set(CMAKE_CXX_STANDARD 17)

# 设置OpenCV和Eigen的路径
find_package(OpenCV 4 REQUIRED)# 加入dnn模块
find_package(absl REQUIRED)
#find_package(Ceres REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
#include_directories(${CERES_INCLUDE_DIRS})
include_directories("/usr/include/eigen3")
#include_directories("/usr/include/ceres")
include_directories("/opt/DahuaTech/MVviewer/include")


# List header files (not mandatory, but can help with IDE support)
set(HEADER_FILES
        include/module/TorosamyModule.hpp
        include/pod/multiton/camera/VideoCamera.hpp
        include/pod/multiton/camera/VirtualCamera.hpp
        include/pod/singleton/SerialPort.hpp
        include/pod/multiton/package/SendDataPacket.hpp
        include/pod/multiton/package/ReceiveDataPacket.hpp
        include/pod/multiton/package/DataPacket.hpp
        include/pod/multiton/camera/Camera.hpp
        include/pod/multiton/camera/MindCamera.hpp
        #include/pod/multiton/camera/DahuaCamera.hpp
        include/pod/multiton/package/DataPacket.hpp
        include/utils/MessageUtils.hpp
        include/utils/BulletUtils.hpp
        include/pod/singleton/manager/ModuleManager.hpp
        include/pod/singleton/manager/CameraManager.hpp
        include/pod/singleton/manager/PacketManager.hpp
        include/module/BuffModule.hpp

        include/Robot.hpp
        include/module/test/TestModule.hpp

)

# List source files
set(SOURCE_FILES
        main.cpp
        src/Robot.cpp
        src/module/TorosamyModule.cpp
        src/pod/multiton/camera/VideoCamera.cpp
        src/pod/multiton/camera/VirtualCamera.cpp
        src/pod/singleton/SerialPort.cpp
        src/pod/multiton/camera/Camera.cpp
        src/pod/multiton/camera/MindCamera.cpp
        #src/pod/multiton/camera/DahuaCamera.cpp
        src/pod/singleton/manager/CameraManager.cpp
        src/utils/MessageUtils.cpp
        src/utils/BulletUtils.cpp
        src/pod/singleton/manager/ModuleManager.cpp
        src/pod/multiton/package/ReceiveDataPacket.cpp
        src/pod/multiton/package/SendDataPacket.cpp
        src/module/BuffModule.cpp
        src/module/test/TestModule.cpp
        src/pod/multiton/package/DataPacket.cpp
        include/pod/singleton/manager/PacketManager.hpp
        src/pod/singleton/manager/PacketManager.cpp
)

# Add executable
add_executable(rune ${SOURCE_FILES} ${HEADER_FILES}

    include/module/buff/Detector/Detector.hpp
    include/module/buff/Detector/BuffArmor.hpp
    include/module/buff/Manager/BuffManager.hpp
    include/module/buff/Manager/EkfManager.hpp
    include/module/buff/PredictSolver/ExtendedKalman.hpp
    include/module/buff/PredictSolver/GaussNewtonSolver.hpp
    include/module/buff/PredictSolver/tracker.hpp
    include/module/buff/PredictSolver/DFtransform.hpp
    #include/module/buff/PoseSolver/BuffSolver.hpp
    include/module/inference/YOLO11module.hpp
    src/module/buff/Detector/Detector.cpp
    src/module/buff/Detector/BuffArmor.cpp
    src/module/buff/Manager/BuffManager.cpp
    src/module/buff/Manager/EkfManager.cpp
    src/module/buff/PredictSolver/ExtendedKalman.cpp
    src/module/buff/PredictSolver/GaussNewtonSolver.cpp
    src/module/buff/PredictSolver/tracker.cpp
    src/module/buff/PredictSolver/DFtransform.cpp
    #src/module/buff/PoseSolver/BuffSolver.cpp
    src/module/inference/YOLO11Module.cpp
    
   

)

#include_directories(
#            ${OpenCV_INCLUDE_DIRS}
#            /opt/intel/openvino_2024_new/runtime/include
#)

# target_link_libraries(${PROJECT_NAME}
#            ${OpenCV_LIBS}
#            /opt/intel/openvino_2024_new/runtime/lib/intel64/libopenvino.so
# )


# Link libraries
target_link_libraries(rune
        ${OpenCV_LIBS}      # OpenCV库变量
        ${EIGEN3_LIBS}
#        ${CERES_LIBRARIES}
        -lpthread           # 线程库
        -lMVSDK             # 自定义库
        #-lDahuaSDK
        /usr/local/lib/libopencv_core.so
        /usr/local/lib/libopencv_imgproc.so
        /usr/local/lib/libopencv_highgui.so
        /usr/local/lib/libopencv_imgcodecs.so
        /usr/local/lib/libopencv_dnn.so
        /usr/local/lib/libopencv_video.so
        /usr/local/lib/libopencv_videoio.so
        #/usr/local/lib/libopencv_world.so.4.9.0
        #/opt/DahuaTech/MVviewer/lib/libImageConvert.so
        #/opt/intel/openvino_2024_new/runtime/lib/intel64/libopenvino.so
        # 列出其他OpenCV库文件
)



#cmake_minimum_required(VERSION 3.10)
#project(robot-hero)

## 使用 C++17 标准
#set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
#find_package(OpenCV REQUIRED QUIET)
#find_package(Eigen3 REQUIRED QUIET)



#set(LIB_DIRECTORIES
#        ${CMAKE_CURRENT_SOURCE_DIR}/resource/DahuaCamera/lib
#        ${CMAKE_CURRENT_SOURCE_DIR}/resource/DahuaCamera/lib/GenICam/bin/Linux64_x64
#        ${CMAKE_CURRENT_SOURCE_DIR}/resource/MindCamera/lib/amd64
#        #resource/MindCamera/lib/arm64
#)
#link_directories(${LIB_DIRECTORIES})




#file(GLOB_RECURSE SOURCE_FILES
#        "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp"
#        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
#        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
#)
#add_executable(${PROJECT_NAME} ${SOURCE_FILES})



## 继续设置其他目标选项
#target_link_libraries(${PROJECT_NAME} PUBLIC
#        ${OpenCV_LIBS}
#        ${EIGEN3_LIBRARIES}
#        #Dahua
#        ImageConvert
#        MVSDK
#)



## 添加包含目录
#set(INCLUDE_DIRECTORIES
#        ${CMAKE_CURRENT_SOURCE_DIR}/include
#        resource/DahuaCamera/include
#        resource/MindCamera/include
#)
#target_include_directories(${PROJECT_NAME} PUBLIC
#        ${INCLUDE_DIRECTORIES}
#        ${OpenCV_INCLUDE_DIRS}
#        ${EIGEN3_INCLUDE_DIR}
#)

## 设置输出目录
#set_target_properties(${PROJECT_NAME} PROPERTIES
#        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin/default
#        RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/bin/debug
#        RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/bin/release
#)


