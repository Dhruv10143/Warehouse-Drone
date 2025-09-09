import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='usb_cam',
            name='usb_cam',
            executable='usb_cam_node_exe',
            output='screen',
            parameters=[{
                'video_device': '/dev/video4',
                'image_width': 1920,
                'image_height': 1080,
                'pixel_format': 'mjpeg2rgb',
                'io_method': 'mmap',
                'framerate': 30.0,
                'camera_frame_id': 'usb_cam',
                'av_device_format': 'YUV422P',
            }]
        ),

        Node(
            package='whycon',
            name='whycon',
            namespace='whycon',
            executable='whycon',
            output='screen',
            parameters=[{
                'targets': 1,
                'name': 'whycon',
                'outer_diameter': 0.38,
                'inner_diameter': 0.14,
            }]
        ),

        Node(
            package='image_view',
            executable='image_view',
            namespace='whycon_display',
            name='image_view',
            output='screen',
            remappings=[
                ('image', '/whycon/image_out')
            ]
        ),

        Node(
            package='crsf_ros2',
            executable='crsf_ros',
            name='crsf_ros',
            output='screen'
        ),

        Node(
            package='swift_pico_hw',
            executable='pico_controller',
            name='pico_controller',
            output='screen'
        ),

        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', 'task_4a', '/whycon/poses'],
            output='screen'
        )

    ])
