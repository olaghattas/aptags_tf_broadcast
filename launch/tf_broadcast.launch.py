from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()

    aptags_file = DeclareLaunchArgument(
        "hewitthall_aptags",
        default_value="/home/olagh/smart-home/src/smart-home/external/aptags_tf_broadcast/config/hewitthall_aptags.yaml",
        description="aptags location"
    )
    ld.add_action(aptags_file)
    aptags = Node(
        package="aptags_tf_broadcast",
        executable="aptag_broadcast_node",
        name="aptags_launch",
        parameters=[
            {"yaml_file_name": LaunchConfiguration("hewitthall_aptags")}
        ]
    )

    ld.add_action(aptags)

    room_file = DeclareLaunchArgument(
        "rooms_location",
        default_value="/home/olagh/smart-home/src/smart-home/external/aptags_tf_broadcast/config/rooms.yaml",
        description="rooms location"
    )
    ld.add_action(room_file)

    rooms = Node(
        package="aptags_tf_broadcast",
        executable="aptag_broadcast_node",
        name="rooms_launch",
        parameters=[
            {"yaml_file_name": LaunchConfiguration("rooms_location")}
        ]
    )

    ld.add_action(rooms)

    helper_file = DeclareLaunchArgument(
        "helper_location",
        default_value="/home/olagh/smart-home/src/smart-home/external/aptags_tf_broadcast/config/helpers.yaml",
        description="helper location"
    )

    ld.add_action(helper_file)

    helpers = Node(
        package="aptags_tf_broadcast",
        executable="aptag_broadcast_node",
        name="helper_launch",
        parameters=[
            {"yaml_file_name": LaunchConfiguration("helper_location")}
        ]
    )

    ld.add_action(helpers)

    # tf_mat_file = DeclareLaunchArgument(
    #     "tf_mat_Lo",
    #     default_value="/home/olagh/smart-home/src/smart-home/external/aptags_tf_broadcast/config/transformation_matrix.yaml",
    #     description="tf_mat"
    # )
    #
    # ld.add_action(tf_mat_file)
    #
    # tf_mat = Node(
    #     package="aptags_tf_broadcast",
    #     executable="aptag_broadcast_node",
    #     name="tf_mat",
    #     parameters=[
    #         {"yaml_file_name": LaunchConfiguration("tf_mat_Lo")}
    #     ]
    # )
    #
    # ld.add_action(tf_mat)
    return ld
