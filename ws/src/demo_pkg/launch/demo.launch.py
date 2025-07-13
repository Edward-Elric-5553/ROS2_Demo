import launch
import launch_ros


def generate_launch_description():
    
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


    return launch.LaunchDescription([
        topic_publisher_node,
        topic_subscriber_node
    ])