import launch
import launch_ros


def generate_launch_description():
    """
    topic_publisher_node = launch_ros.actions.Node(
        package="demo_pkg",
        executable="topic_publisher",
        output="both"
    )

    topic_subscriber_node = launch_ros.actions.Node(
        package="demo_pkg",
        executable="topic_subscriber",
        output="both"
    )
    """

    service_server_node = launch_ros.actions.Node(
        package="demo_pkg",
        executable="service_server",
        output="both"
    )

    service_client_node = launch_ros.actions.Node(
        package="demo_pkg",
        executable="service_client",
        output="both"
    )

    return launch.LaunchDescription([
        service_server_node,
        service_client_node
    ])