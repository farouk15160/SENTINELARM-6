import os
Import("env")

# Path to your ROS2 workspace
ros2_ws_path = os.path.expanduser("~/ros2_ws")

# Command to build micro-ROS with your custom packages
build_command = f'ros2 run micro_ros_setup create_firmware_ws.sh generate_lib --ros2-ws {ros2_ws_path}'

def build_microros_lib(source, target, env):
    print("Building micro-ROS library with custom interfaces...")
    env.Execute(build_command)

# Add a custom build target "build_microros" and run it before the main build
env.AddPreAction("buildprog", build_microros_lib)
env.AddTarget("build_microros", None, build_microros_lib)