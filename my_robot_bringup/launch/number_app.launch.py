from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    number_publisher = Node(
        package="ros2_pub_sub",
        executable="number_publisher",
        name="my_number_publisher",
        remappings=[("/number","/my_number")]
    )

    number_counter = Node(
        package = "ros2_pub_sub",
        executable="number_subscriber",
        name="my_number_subscriber",
        remappings=[("/number","/my_number")]
    )

    ld.add_action(number_publisher)
    ld.add_action(number_counter)


    return ld
