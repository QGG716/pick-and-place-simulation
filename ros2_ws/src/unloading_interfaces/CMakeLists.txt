cmake_minimum_required(VERSION 3.8)
project(unloading_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(trajectory_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CargoObservation.msg"
  "msg/ControllerStopFact.msg"
  "msg/ExecutionAuthorization.msg"
  "msg/ExecutionCancel.msg"
  "msg/ExecutionContext.msg"
  "msg/ExecutionEvent.msg"
  "msg/ExecutionGrant.msg"
  "msg/MechanismState.msg"
  "msg/PerceptionObservation.msg"
  "msg/PlanningWorldSnapshot.msg"
  "msg/RgbdCaptureMetadata.msg"
  "msg/StopAcknowledgement.msg"
  "msg/UnknownRegion.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs trajectory_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
