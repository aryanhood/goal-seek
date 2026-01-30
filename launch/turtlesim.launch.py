from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart, OnProcessExit

def generate_launch_description():
    turtlesim = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="sim"
    )

    teleport = ExecuteProcess(
        cmd=[
            "ros2", "service", "call", "/turtle1/teleport_absolute", "turtlesim/srv/TeleportAbsolute", "{x: 1.0, y: 1.0, theta: 0.0}"
        ],
        output="screen"
    )

    clear = ExecuteProcess(
        cmd=[
            "ros2", "service", "call", "/clear", "std_srvs/srv/Empty", "{}"
        ],
        output="screen"
    )

    teleport_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=turtlesim,
            on_start=[teleport],
        )
    )

    clear_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=teleport,
            on_exit=[clear],
        )
    )

    odom_node = Node(
        package="goal_seek",
        executable="odom_publisher",
        name="odom_publisher"
    )

    odom_publisher_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=clear,
            on_exit=[odom_node],
        )
    )

    return LaunchDescription([
        turtlesim,
        teleport_handler,
        clear_handler,
        odom_publisher_handler
    ])
