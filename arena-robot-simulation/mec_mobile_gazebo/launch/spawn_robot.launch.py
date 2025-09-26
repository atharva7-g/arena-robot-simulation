import os
from pydoc import describe
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_urdf_path = get_package_share_directory("mec_mobile_description")
    pkg_gazebo_path = get_package_share_directory("mec_mobile_gazebo")

    gazebo_models_path, ignore_last_dir = os.path.split(pkg_urdf_path)
    # os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path

    rviz_launch_arg = DeclareLaunchArgument(
        "rviz", default_value="true", description="Open RViz."
    )

    world_arg = DeclareLaunchArgument(
        "world",
        default_value="empty.sdf",
        description="Name of the Gazebo world file to load",
    )

    model_arg = DeclareLaunchArgument(
        "model",
        default_value="robot_3d.urdf.xacro",
        description="Name of the URDF description to load",
    )

    sarm_arg = DeclareLaunchArgument(
        "sarm",
        default_value="sarm.urdf.xacro",
        description="Name of the URDF description to load",
    )

    # Define the path to your URDF or Xacro file
    urdf_file_path = PathJoinSubstitution(
        [
            pkg_urdf_path,  # Replace with your package name
            "urdf",
            "robots",
            LaunchConfiguration("model"),  # Replace with your URDF or Xacro file
        ]
    )

    sarm_file_path = PathJoinSubstitution(
        [pkg_urdf_path, "urdf", "sarm", LaunchConfiguration("sarm")]
    )

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_path, "launch", "world.launch.py"),
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
        }.items(),
    )

    # Launch rviz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(pkg_urdf_path, "rviz", "rviz.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[
            {"use_sim_time": True},
        ],
    )

    # Spawn the URDF model using the `/world/<world_name>/create` service
    spawn_urdf_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "lfr",
            "-topic",
            "robot_description",
            "-x",
            "-1.94",
            "-y",
            "0.78",
            "-z",
            "0.2",
            "-Y",
            "-1.6",  # Initial spawn position
        ],
        output="screen",
        parameters=[
            {"use_sim_time": True},
        ],
    )

    spawn_sarm_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "sarm",
            "-topic",
            "/sarm_description",
            "-x",
            "1.0",
            "-y",
            "0.0",
            "-z",
            "0.2",  # Initial position
            "-R",
            "-1.5708",  # Roll = -π/2
            "-P",
            "0.0",  # Pitch = 0
            "-Y",
            "0.0",  # Yaw = 0
        ],
        output="screen",
        parameters=[
            {"use_sim_time": True},
        ],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": Command(["xacro", " ", urdf_file_path]),
                "use_sim_time": True,
            },
        ],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    sarm_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="sarm_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": Command(["xacro ", sarm_file_path]),
                "use_sim_time": True,
            },
        ],
        remappings=[
            (
                "robot_description",
                "sarm_description",
            ),  # Remap to sarm_description topic
            # Uncomment these if you want separate TF trees:
            # ('/tf', 'sarm/tf'),
            # ('/tf_static', 'sarm/tf_static')
        ],
    )

    # Node to bridge messages like /cmd_vel and /odom
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            "/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
            "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
            "/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            "/cam_1/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/cam_1/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            "/sarm_description@std_msgs/msg/String@gz.msgs.StringMsg"
        ],
        output="screen",
        parameters=[
            {"use_sim_time": True},
        ],
    )

    # joint_state_publisher_gui_node = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    # )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(rviz_launch_arg)
    launchDescriptionObject.add_action(world_arg)
    launchDescriptionObject.add_action(model_arg)
    launchDescriptionObject.add_action(sarm_arg)
    launchDescriptionObject.add_action(world_launch)
    # launchDescriptionObject.add_action(rviz_node)
    launchDescriptionObject.add_action(spawn_urdf_node)
    launchDescriptionObject.add_action(robot_state_publisher_node)
    launchDescriptionObject.add_action(gz_bridge_node)
    launchDescriptionObject.add_action(sarm_state_publisher_node)
    launchDescriptionObject.add_action(spawn_sarm_node)

    return launchDescriptionObject
