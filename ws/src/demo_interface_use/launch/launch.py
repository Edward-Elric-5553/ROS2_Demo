import launch
import launch_ros


def generate_launch_description():

    dmeo_robot_node = launch_ros.actions.Node(
        package="demo_interface_use",
        executable="demo_robot_node",
        output="both"
    )

    demo_control_node = launch_ros.actions.Node(
        package="demo_interface_use",
        executable="demo_control_node",
        output="both"
    )

    return launch.LaunchDescription([
        dmeo_robot_node,
        demo_control_node
    ])