from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("namespace", default_value="kinect"))

    namespace = LaunchConfiguration("namespace")

    return LaunchDescription([
        ComposableNodeContainer(
            name="kinect_component",
            namespace=namespace,
            package="rclcpp_components",
            # MAYDO: switch to component_container_mt and provide "thread_num" parameter
            executable="component_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="kinect_stack2",
                    plugin="KinectNode",
                    name="kinect_node",
                    namespace=namespace,
                    extra_arguments=[{"use_intra_process_comms": True}]
                ),
                ComposableNode(
                    package="depth_image_proc",
                    plugin="depth_image_proc::ConvertMetricNode",
                    name="kinect_convert_metric",
                    namespace=namespace,
                    remappings=[
                        ("image_raw", "depth/image_raw"),
                        ("image", "depth/image_metric")
                    ],
                    extra_arguments=[{"use_intra_process_comms": True}]
                ),
                ComposableNode(
                    package="depth_image_proc",
                    plugin="depth_image_proc::RegisterNode",
                    name="kinect_depth_register",
                    namespace=namespace,
                    remappings=[
                        # rgb/camera_info
                        # depth/camera_info
                        ("depth/image_rect", "depth/image_metric")
                    ],
                    extra_arguments=[{"use_intra_process_comms": True}]
                ),
                # There was a problem with calculating the xyzrgb pcl in the same container.
                # This is most likely due to the single threaded containter used.
                # For now, just spawning the component as a node outside seems to work fine.
                # If in future the performance is greatly degraded,
                # consider handling the MAYDO above and re-integrate the component
                # into a multithreaded container.

                # ComposableNode(
                #     package="depth_image_proc",
                #     plugin="depth_image_proc::PointCloudXyzrgbNode",
                #     name="kinect_rgb_pcl",
                #     namespace=namespace,
                #     remappings=[
                #         # rgb/camera_info
                #         # depth/camera_info
                #         ("rgb/image_rect_color", "rgb/image_raw")
                #     ],
                #     extra_arguments=[{"use_intra_process_comms": True}]
                # ),
            ]
        ),
        # See comment above (:52) for why this is its own node.
        Node(
            package="depth_image_proc",
            executable="point_cloud_xyzrgb_node",
            namespace=namespace,
            remappings=[
                # rgb/camera_info
                # depth/camera_info
                ("rgb/image_rect_color", "rgb/image_raw")
            ],
        ),
    ])
