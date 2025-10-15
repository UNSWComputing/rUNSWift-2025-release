cmake_minimum_required(VERSION 3.8)
project(vision)

# --- Standard Compiler Options and Settings ---
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# --- Find All Required Dependencies ---
# ROS 2 Packages
find_package(ament_cmake REQUIRED)
find_package(runswift_interfaces REQUIRED)
find_package(usb_cam REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nao_lola_command_msgs REQUIRED)
find_package(nao_lola_sensor_msgs REQUIRED)

# System Libraries
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(HDF5 REQUIRED)
find_package(Protobuf REQUIRED)

message(STATUS "--- HDF5 Debug Info ---")
message(STATUS "HDF5 Found: ${HDF5_FOUND}")
message(STATUS "HDF5 Include Dirs: ${HDF5_INCLUDE_DIRS}")
message(STATUS "HDF5 Libraries: ${HDF5_LIBRARIES}")
message(STATUS "-----------------------")
# --------------------------------

# --- Testing Configuration (Unchanged) ---
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

# --- Define Paths and Source Files for CompiledNN ---
set(COMPILEDNN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/CompiledNN)

# This list is copied directly from the CompiledNN CMakeLists.txt
set(COMPILEDNN_SOURCES
    ${COMPILEDNN_DIR}/Src/CompiledNN/CompiledNN.cpp
    ${COMPILEDNN_DIR}/Src/CompiledNN/Model.cpp
    ${COMPILEDNN_DIR}/Src/CompiledNN/SimpleNN.cpp
    ${COMPILEDNN_DIR}/Src/CompiledNN/CompiledNN/ActivationFunctions.cpp
    ${COMPILEDNN_DIR}/Src/CompiledNN/CompiledNN/CompilationSettings.cpp
    ${COMPILEDNN_DIR}/Src/CompiledNN/CompiledNN/Operations/Activation.cpp
    ${COMPILEDNN_DIR}/Src/CompiledNN/CompiledNN/Operations/Arithmetic.cpp
    ${COMPILEDNN_DIR}/Src/CompiledNN/CompiledNN/Operations/BatchNormalization.cpp
    ${COMPILEDNN_DIR}/Src/CompiledNN/CompiledNN/Operations/Concatenate.cpp
    ${COMPILEDNN_DIR}/Src/CompiledNN/CompiledNN/Operations/Conv1D.cpp
    ${COMPILEDNN_DIR}/Src/CompiledNN/CompiledNN/Operations/Conv2D.cpp
    ${COMPILEDNN_DIR}/Src/CompiledNN/CompiledNN/Operations/Cropping2D.cpp
    ${COMPILEDNN_DIR}/Src/CompiledNN/CompiledNN/Operations/DConv2D.cpp
    ${COMPILEDNN_DIR}/Src/CompiledNN/CompiledNN/Operations/Dense.cpp
    ${COMPILEDNN_DIR}/Src/CompiledNN/CompiledNN/Operations/GlobalPooling2D.cpp
    ${COMPILEDNN_DIR}/Src/CompiledNN/CompiledNN/Operations/Im2Col2D.cpp
    ${COMPILEDNN_DIR}/Src/CompiledNN/CompiledNN/Operations/Pooling1D.cpp
    ${COMPILEDNN_DIR}/Src/CompiledNN/CompiledNN/Operations/Pooling2D.cpp
    ${COMPILEDNN_DIR}/Src/CompiledNN/CompiledNN/Operations/QuantizedInputConvStrided4x4WithReLU.cpp
    ${COMPILEDNN_DIR}/Src/CompiledNN/CompiledNN/Operations/Softmax.cpp
    ${COMPILEDNN_DIR}/Src/CompiledNN/CompiledNN/Operations/UInt8Input.cpp
    ${COMPILEDNN_DIR}/Src/CompiledNN/CompiledNN/Operations/UpSampling2D.cpp
    ${COMPILEDNN_DIR}/Src/CompiledNN/CompiledNN/Operations/ZeroPadding1D.cpp
    ${COMPILEDNN_DIR}/Src/CompiledNN/CompiledNN/Operations/ZeroPadding2D.cpp
    ${COMPILEDNN_DIR}/Src/CompiledNN/CompiledNN/Util/ExpApprox.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/arm/a64assembler.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/arm/a64builder.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/arm/a64compiler.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/arm/a64emithelper.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/arm/a64formatter.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/arm/a64func.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/arm/a64instapi.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/arm/a64instdb.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/arm/a64operand.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/arm/a64rapass.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/arm/armformatter.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/core/archtraits.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/core/assembler.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/core/builder.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/core/codeholder.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/core/codewriter.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/core/compiler.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/core/constpool.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/core/cpuinfo.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/core/emithelper.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/core/emitter.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/core/emitterutils.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/core/environment.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/core/errorhandler.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/core/formatter.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/core/func.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/core/funcargscontext.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/core/globals.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/core/inst.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/core/jitallocator.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/core/jitruntime.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/core/logger.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/core/operand.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/core/osutils.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/core/ralocal.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/core/rapass.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/core/rastack.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/core/string.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/core/support.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/core/target.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/core/type.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/core/virtmem.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/core/zone.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/core/zonehash.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/core/zonelist.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/core/zonestack.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/core/zonetree.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/core/zonevector.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/x86/x86assembler.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/x86/x86builder.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/x86/x86compiler.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/x86/x86emithelper.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/x86/x86formatter.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/x86/x86func.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/x86/x86instapi.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/x86/x86instdb.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/x86/x86operand.cpp
    ${COMPILEDNN_DIR}/3rdParty/asmjit/src/asmjit/x86/x86rapass.cpp
    ${COMPILEDNN_DIR}/3rdParty/B-Human/Platform/BHAssert.cpp
    ${COMPILEDNN_DIR}/3rdParty/B-Human/Streaming/InOut.cpp
    ${COMPILEDNN_DIR}/3rdParty/B-Human/Streaming/InStreams.cpp
    ${COMPILEDNN_DIR}/3rdParty/B-Human/Streaming/SimpleMap.cpp
    ${COMPILEDNN_DIR}/Src/CompiledNN/Formats/KerasHDF5.cpp
    ${COMPILEDNN_DIR}/Src/CompiledNN/Formats/ONNX.cpp
)

# Generate protobuf files for ONNX support
protobuf_generate_cpp(ONNX_PROTO_SOURCES ONNX_PROTO_HEADERS ${COMPILEDNN_DIR}/3rdParty/onnx/onnx.proto)
# Add the generated files to our source list
list(APPEND COMPILEDNN_SOURCES ${ONNX_PROTO_SOURCES} ${ONNX_PROTO_HEADERS})

# --- Set Global Include Directories ---
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  # Add the include directories needed by CompiledNN
  ${HDF5_INCLUDE_DIRS}
  ${COMPILEDNN_DIR}/Src
  ${COMPILEDNN_DIR}/3rdParty/B-Human
  ${COMPILEDNN_DIR}/3rdParty/asmjit/src
  ${HDF5_LIBRARIES}
  # Include dir for generated protobuf headers
  ${CMAKE_CURRENT_BINARY_DIR}
)

# --- Define Existing Executables (Unchanged) ---
add_executable(Preprocessing src/Preprocessing.cpp)
ament_target_dependencies(Preprocessing sensor_msgs runswift_interfaces rclcpp cv_bridge OpenCV)

add_executable(BallDetector src/BallDetector.cpp)
ament_target_dependencies(BallDetector sensor_msgs runswift_interfaces rclcpp cv_bridge vision_msgs tf2 tf2_ros tf2_geometry_msgs OpenCV)

add_executable(BotSegmentation src/BottomSegmentation.cpp ${COMPILEDNN_SOURCES})
ament_target_dependencies(BotSegmentation sensor_msgs runswift_interfaces rclcpp cv_bridge vision_msgs tf2 tf2_ros tf2_geometry_msgs)
target_link_libraries(BotSegmentation
                    ${OpenCV_LIBRARIES}
                    ${HDF5_LIBRARIES}
                    ${Protobuf_LIBRARIES}
                    $<$<PLATFORM_ID:Linux>:pthread>
                    $<$<PLATFORM_ID:Linux>:rt>)

target_compile_definitions(BotSegmentation PRIVATE
    # These definitions are copied from the CompiledNN CMakeLists.txt
    ASMJIT_STATIC ASMJIT_NO_AARCH32 ASMJIT_NO_AARCH64 ASMJIT_NO_BUILDER ASMJIT_NO_COMPILER
    ASMJIT_NO_DEPRECATED ASMJIT_NO_INTROSPECTION ASMJIT_NO_LOGGING ASMJIT_NO_TEXT ASMJIT_NO_VALIDATION
    WITH_KERAS_HDF5
    WITH_ONNX
)

add_executable(CameraFlip src/CameraFlip.cpp)
ament_target_dependencies(CameraFlip rclcpp)

add_executable(Unwrap src/unwrap/Unwrap.cpp)
ament_target_dependencies(Unwrap rclcpp cv_bridge OpenCV tf2 tf2_ros tf2_geometry_msgs sensor_msgs visualization_msgs)

# add_executable(FeatureDetector src/unwrap/FeatureDetector.cpp)
# target_sources(FeatureDetector PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/unwrap/FieldTraceHelpers.cpp>)
# ament_target_dependencies(FeatureDetector rclcpp sensor_msgs cv_bridge OpenCV tf2 tf2_geometry_msgs runswift_interfaces)

add_executable(CircleOnlyDetector src/unwrap/CircleOnlyDetector.cpp)
target_sources(CircleOnlyDetector PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/unwrap/FieldTraceHelpers.cpp>)
ament_target_dependencies(CircleOnlyDetector rclcpp sensor_msgs cv_bridge OpenCV tf2 tf2_geometry_msgs
    runswift_interfaces nao_lola_sensor_msgs nao_lola_command_msgs)

# --- Install All Targets ---
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
    Preprocessing
    BallDetector
    CameraFlip
    Unwrap
#    FeatureDetector
    CircleOnlyDetector
    BotSegmentation
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/eye_leds_control.py
    scripts/vision_debug_cca.py
    scripts/ball_detector_model.py
    scripts/vision_pipeline_switch_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
